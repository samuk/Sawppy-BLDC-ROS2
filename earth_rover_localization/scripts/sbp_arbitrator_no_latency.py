#!/usr/bin/env python

#https://stackoverflow.com/questions/34337514/updated-variable-into-multiprocessing-python
#https://stackoverflow.com/questions/44219288/should-i-bother-locking-the-queue-when-i-put-to-or-get-from-it/44219646
import rospy
import os
import datetime
import subprocess
import json
import sbp.msg
import sys
import time
import operator

import threading, queue
from multiprocessing import Process, Value
from multiprocessing.managers import BaseManager

from sbp.table import dispatch
from sbp.client.loggers.udp_logger import UdpLogger
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
import argparse

# NTRIP host
NTRIP_HOST = rospy.get_param('/sbp_arbitrator/ntrip_host', "rtk2go.com")
NTRIP_PORT = rospy.get_param('/sbp_arbitrator/ntrip_port', 2101)
NTRIP_MOUNT_POINT = rospy.get_param('/sbp_arbitrator/ntrip_mount_point', "ER_Valldoreix_1")
#RADIO
RADIO_PORT =  rospy.get_param('/sbp_arbitrator/radio_port', "/dev/freewaveGXMT14")
RADIO_BAUDRATE = rospy.get_param('/sbp_arbitrator/radio_baudrate', 115200)
# UDP LOGGER
UDP_ADDRESS = rospy.get_param('/sbp_arbitrator/udp_address', "192.168.8.222")
UDP_PORT =  rospy.get_param('/sbp_arbitrator/udp_port', 55558)

# create instance of UdpLogger object
udp = UdpLogger(UDP_ADDRESS, UDP_PORT)

# get current year:month:day:hour
def get_current_time():
   now = datetime.datetime.now(datetime.timezone.utc)
   return "{}:{}:{}:{}".format(now.year, now.month, now.day, now.hour)

def ntrip_corrections(q_ntrip):
    # run command to listen to ntrip client, convert from rtcm3 to sbp and from sbp to json redirecting the stdout
    str2str_cmd = ["str2str", "-in", "ntrip://{}:{}/{}".format(NTRIP_HOST, NTRIP_PORT, NTRIP_MOUNT_POINT)]
    rtcm3tosbp_cmd = ["rtcm3tosbp", "-d", get_current_time()]
    cmd = "{} 2>/dev/null| {} | sbp2json".format(' '.join(str2str_cmd), ' '.join(rtcm3tosbp_cmd))
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    while not rospy.is_shutdown():
        line = p.stdout.readline().strip()
        try:
            json_msg = json.loads(line)
            # handle encoded JSON
            if 'data' in json_msg and 'payload' in json_msg['data']:
                json_msg = json_msg['data']
        except ValueError:
            continue

        # sanity check
        if 'msg_type' not in json_msg:
            continue

        # parse sbp msgs
        sbp_msg = sbp.msg.SBP.from_json_dict(json_msg)

        if ntrip_sender is None:
            ntrip_sender = sbp_msg.sender # get ntrip sender id

        if sbp_msg.msg_type == sbp.observation.SBP_MSG_OBS:
            q_ntrip.put(sbp_msg)
        else:
            udp.call(sbp_msg)

def radio_corrections(q_radio):
    global radio_sender # get radio sender id
    with PySerialDriver(RADIO_PORT, baud=RADIO_BAUDRATE) as driver:
        print(driver.read)
        with Handler(Framer(driver.read, None, verbose=False)) as source:
            try:
                for sbp_msg, metadata in source.filter():
                    if radio_sender is None:
                        radio_sender = sbp_msg.sender # get ntrip sender id

                    if sbp_msg.msg_type == sbp.observation.SBP_MSG_OBS:
                        q_radio.put(sbp_msg)
                    else:
                        if ntrip_sender is not None:
                            sbp_msg.sender = ntrip_sender
                        udp.call(sbp_msg)

            except KeyboardInterrupt:
                pass

def get_queue_msgs(queue):
    msg_list = []
    while not queue.empty():
         msg_list.append(queue.get())
    return msg_list

def get_packet_index(msg):
    packet = hex(msg.header.n_obs)
    return int(packet[2]), int(packet[3])

def old_msg_cond(msg, tow, ind):
    packet_seq, packet_index = get_packet_index(msg)
    if msg.header.t.tow < prev_tow or (msg.header.t.tow == tow and packet_index<=ind):
        return True
    else:
        return False

def check_existing_msgs(msg_list, new_msg, prev_tow, prev_packet_index):
    existing_message = False
    if not msg_list:
        msg_list.append(new_msg)
    else:
        for msg in msg_list:
            [_, msg_packet_index] = get_packet_index(msg)
            [_, new_msg_packet_index] = get_packet_index(new_msg)
            if msg.header.t.tow == new_msg.header.t.tow and new_msg_packet_index == msg_packet_index:
                existing_message = True
                if msg.payload != new_msg.payload: #sanity check
                    rospy.logwarn("Received 2 equivalent messages with different PAYLOAD")
                break
        if existing_message == False:
            msg_list.append(new_msg)
    return msg_list

def get_sender(msg):
    if msg.sender == ntrip_sender:
        sender = "Ntrip"
    elif msg.sender == radio_sender:
        sender = "Radio"
    else:
        sender = str(msg.sender)
        rospy.logwarn("Wrong sender definition: " + sender)
    return sender

def send_and_print_msg(msg):
    sender = get_sender(msg)
    if sender == "Radio" and ntrip_sender is not None:
        msg.sender = ntrip_sender
    rospy.loginfo(sender + ", " + str(msg.header.t.tow) + ", " + str(packet_index) + ", " + str(packet_seq))
    udp.call(msg)


if __name__ == '__main__':
    rospy.init_node('sbp_arbitrator', anonymous=True)

    ntrip_sender = None
    radio_sender = None
    q_ntrip = queue.Queue()
    q_radio = queue.Queue()

    th1 = threading.Thread(target=ntrip_corrections,args=(q_ntrip,))
    th2 = threading.Thread(target=radio_corrections,args=(q_radio,))

    th1.start()
    th2.start()

    msgs_to_evaluate = []
    ntrip_msgs = []
    radio_msgs = []
    prev_tow = 0
    prev_packet_index = 0
    expected_packet = 0
    epoch_timeout = 5 # number of seconds to wait until timout

    # Arbitrate
    while not rospy.is_shutdown():
        while len(ntrip_msgs)==0 and len(radio_msgs)==0:
            ntrip_msgs = get_queue_msgs(q_ntrip)
            radio_msgs = get_queue_msgs(q_radio)

        # Evaluate ntrip and radio tow and check if msg is repeated
        for msg in ntrip_msgs + radio_msgs:
            msg = dispatch(msg)
            if msg.header.t.tow >= prev_tow:
                msgs_to_evaluate = check_existing_msgs(msgs_to_evaluate, msg, prev_tow, prev_packet_index)

        # Order messages
        msgs_to_evaluate.sort(key=operator.attrgetter('header.t.tow', 'header.n_obs'))

        # Evaluate msgs to send
        for msg in msgs_to_evaluate:
            packet_seq, packet_index = get_packet_index(msg) # get the index of the packet (starting at 0)
            if msg.header.t.tow == prev_tow and expected_packet!=0:
                if packet_index == expected_packet:
                    send_and_print_msg(msg)
                    prev_packet_index = packet_index
                    prev_tow = msg.header.t.tow
                    if packet_index == packet_seq - 1:
                        expected_packet = 0 #sequence is complete
                        rospy.loginfo("Sequence is complete")
                    else:
                        expected_packet += 1
            elif msg.header.t.tow >= prev_tow:
                if expected_packet == 0 and packet_index == 0: #new epoch
                    send_and_print_msg(msg)
                    prev_packet_index = packet_index
                    expected_packet += 1
                    prev_tow = msg.header.t.tow
                else:
                    diff = abs((prev_tow - msg.header.t.tow)/1000)
                    if diff >= epoch_timeout:
                        rospy.logwarn("Epoch timeout: " + str(diff))
                        expected_packet = 0 # did not receive complete sequence, continue with next epoch after waiting hist_length
        radio_msgs = [] # empty radio msgs for next queue request
        ntrip_msgs = [] # empty ntrip msgs for next queue request
        msgs_to_evaluate[:] = [msg for msg in msgs_to_evaluate if not old_msg_cond(msg, prev_tow, prev_packet_index)] # Update msgs to evaluate
