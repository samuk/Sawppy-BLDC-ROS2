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

from sbp.client.loggers.udp_logger import UdpLogger
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.observation import SBP_MSG_OBS, MsgObs, SBP_MSG_GLO_BIASES, MsgGloBiases, SBP_MSG_BASE_POS_ECEF, MsgBasePosECEF
from sbp.observation import SBP_MSG_EPHEMERIS_BDS, MsgEphemerisBds, SBP_MSG_EPHEMERIS_GAL, MsgEphemerisGal, SBP_MSG_EPHEMERIS_GLO, MsgEphemerisGlo, SBP_MSG_EPHEMERIS_QZSS, MsgEphemerisQzss
#SBP_MSG_EPHEMERIS_GPS, MsgEphemerisGps
import argparse

class MsgObsWithPayload(MsgObs):
    def __init__(self, msg, payload, length, crc):
        super(MsgObsWithPayload, self).__init__(msg)
        self.length = length
        self.payload = payload
        self.crc = crc

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
#def get_current_time():
#    now = datetime.datetime.now(datetime.timezone.utc)
#    return "{}:{}:{}:{}".format(now.year, now.month, now.day, now.hour)

def ntrip_corrections(q_ntrip):
    # run command to listen to ntrip client, convert from rtcm3 to sbp and from sbp to json redirecting the stdout
    str2str_cmd = ["str2str", "-in", "ntrip://{}:{}/{}".format(NTRIP_HOST, NTRIP_PORT, NTRIP_MOUNT_POINT)]
    rtcm3tosbp_cmd = ["rtcm3tosbp"]#, "-d", get_current_time()]
    cmd = "{} 2>/dev/null| {} | sbp2json".format(' '.join(str2str_cmd), ' '.join(rtcm3tosbp_cmd))
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    while True:
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
        sbp_general_msg = sbp.msg.SBP.from_json_dict(json_msg)

        # Mandatory to send msg 72, optional to send msg 117, 137, 138, 139, 141, 142
        if sbp_general_msg.msg_type == (72, 117, 137, 138, 139, 141, 142):
            sbp_general_msg.sender = 0
            udp.call(sbp_general_msg)

        # Arbitrate msg 74
        elif sbp_general_msg.msg_type == 74:
            global ntrip_sender # get ntrip sender id
            if ntrip_sender is None:
                ntrip_sender = sbp_general_msg.sender
            q_ntrip.put(sbp_general_msg)
            #e.set()
            #print("Setting event")
            #print("Putting ntrip msg on queue")

def radio_corrections(q_radio):
    sbp_msg_types = [SBP_MSG_OBS, SBP_MSG_GLO_BIASES, SBP_MSG_BASE_POS_ECEF, SBP_MSG_EPHEMERIS_BDS, SBP_MSG_EPHEMERIS_GAL, SBP_MSG_EPHEMERIS_GLO, SBP_MSG_EPHEMERIS_QZSS]
    global radio_sender # get radio sender id
    with PySerialDriver(RADIO_PORT, baud=RADIO_BAUDRATE) as driver:
        print(driver.read)
        with Handler(Framer(driver.read, None, verbose=False)) as source:
            try:
                for sbp_general_msg, metadata in source.filter(sbp_msg_types):
                    if sbp_general_msg.msg_type == (72, 117, 137, 138, 139, 141, 142):
                        sbp_general_msg.sender = 0
                        udp.call(sbp_general_msg)
                    elif sbp_general_msg.msg_type == 74:
                        if radio_sender is None:
                            radio_sender = sbp_general_msg.sender
                        q_radio.put(sbp_general_msg)
                        #e.set()

            except KeyboardInterrupt:
                pass

# def parse_msg(msg):
#     if msg.msg_type == 72:
#         sbp_msg = MsgBasePosECEF(msg)
#     elif msg.msg_type == 74:
#         sbp_msg = MsgObs(msg)
#     elif msg.msg_type == 117:
#         sbp_msg = MsgGloBiases(msg)
#     elif msg.msg_type == 137:
#         sbp_msg = MsgEphemerisBds(msg)
#     #if msg.msg_type == 138: ## cannot compile !!!!!!!!!!!!!!!!!!!!!!!
#     #    sbp_msg = MsgEphemerisGps(sbp_general_msg)
#     elif msg.msg_type == 139:
#         sbp_msg = MsgEphemerisGlo(msg)
#     elif msg.msg_type == 141:
#         sbp_msg = MsgEphemerisGal(msg)
#     elif msg.msg_type == 142:
#         sbp_msg = MsgEphemerisQzss(msg)
#     return sbp_msg

def get_queue_msgs(queue):
    msg_list = []
    while not queue.empty():
         msg_list.append(queue.get())
    return msg_list

def get_packet_index(msg):
    packet = hex(msg.header.n_obs)
    return int(packet[2]), int(packet[3])

def check_existing_msgs(msg_list, new_msg):
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
        rospy.loginfo("Wrong sender definition: " + str(msg.sender))
    return sender


if __name__ == '__main__':
    rospy.init_node('sbp_arbitrator', anonymous=True)
    ntrip_sender = None
    radio_sender = None
    q_ntrip = queue.Queue()
    q_radio = queue.Queue()
    #e = threading.Event()

    th1 = threading.Thread(target=ntrip_corrections,args=(q_ntrip,))
    th2 = threading.Thread(target=radio_corrections,args=(q_radio,))

    th1.start()
    th2.start()

    msgs_to_evaluate = []
    ntrip_msgs = []
    radio_msgs = []
    prev_tow = 0
    prev_length = 0
    expected_packet = 0
    epoch_timeout = 5 # number of seconds to wait until timout
    timeout = False

    # Arbitrate
    while True:
        #e.wait(5)
        #print("Event received")
        #trigger interruptions from ntrip and radio process??
        while len(ntrip_msgs)==0 and len(radio_msgs)==0:
        #while not radio_msgs and ntrip_msgs:
        #while not radio_msgs:
            ntrip_msgs = get_queue_msgs(q_ntrip)
            radio_msgs = get_queue_msgs(q_radio)
            #e.clear()
        # if radio_msgs:
        #     print("============== RADIO MSGS ==========================")
        #     for msg in radio_msgs:
        #         print(str(msg.header.t.tow) + ", " + str(msg.header.n_obs))
        #     print("========================================")


        # if msgs_to_evaluate:
        #     print("================= PREV VALUATE MSGS =======================")
        #     for msg in msgs_to_evaluate:
        #         print(str(msg.header.t.tow) + ", " + str(msg.header.n_obs))
        #     print("========================================")

        #Evaluate ntrip tow and check if msg is repeated
        for msg in ntrip_msgs:
            full_msg = MsgObsWithPayload(msg, msg.payload, msg.length, msg.crc)
            if full_msg.header.t.tow >= prev_tow:
                msgs_to_evaluate = check_existing_msgs(msgs_to_evaluate, full_msg)

            # Evaluate radio tow and check if msg is repeated

        for msg in radio_msgs:
            if msg.header.t.tow >= prev_tow:
                msgs_to_evaluate = check_existing_msgs(msgs_to_evaluate, msg)

        # if msgs_to_evaluate:
        #     print("================= EVALUATE MSGS =======================")
        #     for msg in msgs_to_evaluate:
        #         print(str(msg.header.t.tow) + ", " + str(msg.header.n_obs))
        #     print("========================================")


        # Order messages
        msgs_to_evaluate.sort(key=operator.attrgetter('header.t.tow', 'header.n_obs'))


        # if msgs_to_evaluate:
        #     print("================= EVALUATE MSGS =======================")
        #     for msg in msgs_to_evaluate:
        #         print(get_sender(msg) + ", " + str(msg.header.t.tow) + ", " + str(msg.header.n_obs))
        #     print("========================================")

        # Evaluate msgs to send
        for msg in msgs_to_evaluate:
            packet_seq, packet_index = get_packet_index(msg) # get the index of the packet (starting at 0)
            # if for some reason tow is smaller remove??
            if msg.header.t.tow < prev_tow:
                msgs_to_evaluate.remove(msg)
            if msg.header.t.tow == prev_tow and expected_packet!=0 and timeout==False:
                if packet_index == expected_packet:
                    sender = get_sender(msg)
                    msg.sender = 0
                    udp.call(msg)
                    rospy.loginfo(sender + ", " + str(msg.header.t.tow) + ", " + str(packet_index) + ", " + str(packet_seq))
                    msgs_to_evaluate.remove(msg)
                    if packet_index == packet_seq - 1:
                        expected_packet = 0 #sequence is complete
                        rospy.loginfo("Sequence is complete")
                    else:
                        expected_packet += 1
            elif msg.header.t.tow >= prev_tow:
                timeout = False #move somewhere else?
                if expected_packet == 0 and packet_index == 0: #new epoch
                    sender = get_sender(msg)
                    msg.sender = 0
                    udp.call(msg)
                    rospy.loginfo(sender + ", " + str(msg.header.t.tow) + ", " + str(packet_index) + ", " + str(packet_seq))
                    msgs_to_evaluate.remove(msg) # remove message from list
                    expected_packet += 1
                    prev_tow = msg.header.t.tow
                else:
                    diff = abs((prev_tow - msg.header.t.tow)/1000)
                    if diff >= epoch_timeout and timeout==False:
                        rospy.loginfo("Epoch timeout: " + str(diff))
                        timeout = True
                        expected_packet = 0 # did not receive complete sequence, continue with next epoch after waiting hist_length
        #prev_length = len(msgs_to_evaluate)
        # print("Prev tow" + str(prev_tow))
        radio_msgs = []
        ntrip_msgs = []
