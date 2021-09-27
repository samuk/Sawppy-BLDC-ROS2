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

import threading
import Queue as queue
from multiprocessing import Process, Value
from multiprocessing.managers import BaseManager

from sbp.table import dispatch
from sbp.client.loggers.udp_logger import UdpLogger
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
import argparse
from sbp.observation import SBP_MSG_OBS, MsgObs, SBP_MSG_GLO_BIASES, MsgGloBiases, SBP_MSG_BASE_POS_ECEF, MsgBasePosECEF
from sbp.observation import SBP_MSG_EPHEMERIS_BDS, MsgEphemerisBds, SBP_MSG_EPHEMERIS_GAL, MsgEphemerisGal, SBP_MSG_EPHEMERIS_GLO, MsgEphemerisGlo, SBP_MSG_EPHEMERIS_QZSS, MsgEphemerisQzss
#SBP_MSG_EPHEMERIS_GPS, MsgEphemerisGps

obs_messages = {}

# full time of last message sequence sent via UDP
last_sent_time = None

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
   now = datetime.datetime.utcnow()
   return "{}:{}:{}:{}".format(now.year, now.month, now.day, now.hour)

# convert a MsgObs object into number of seconds since GPS epoch
def get_full_time(msg):
    return msg.header.t.wn * 7 * 24 * 60 * 60 + msg.header.t.tow / 1000

#get msgs from queue object
def get_queue_msgs(queue):
    msg_list = []
    while not queue.empty():
         msg_list.append(queue.get())
    return msg_list

# try and add 'msg' (of type MsgObs) to obs_messages
def obs_message_add(msg):
    full_time = get_full_time(msg)
    if full_time in obs_messages:
        # check for existing message with same value of n_obs field
        matching_msgs = [ m for m in obs_messages[full_time] if m.header.n_obs == msg.header.n_obs ]
        if len(matching_msgs) > 0:
            if matching_msgs[0] != msg:
                print("Sanity check failed on MSG_OBS with tow {}, n_obs {}".format(msg.header.t.tow, msg.header.n_obs))
                return
        else:
            # no existing message, so add to list and resort based on n_obs
            obs_messages[full_time].append(msg)
            obs_messages[full_time].sort(key=lambda m: m.header.n_obs)
    else:
        # no existing messages for this epoch
        obs_messages[full_time] = [msg]


# Check if we have a complete sequence for 'full_time' in obs_messages.
# If so, return the list of MsgObs objects.
def obs_message_get_sequence(full_time):
    if full_time not in obs_messages:
        return None

    msgs = obs_messages[full_time]

    # check current number of packets in the sequence
    if len(msgs) == 0 or len(msgs) != msgs[0].header.n_obs >> 4:
        return None

    # verify sequence numbers
    for pkt_num in range(0, len(msgs)):
        if (msgs[pkt_num].header.n_obs & 0xf) != pkt_num:
            return None

    return msgs

# Remove epochs with times equal to or older than 'full_time' from obs_messages
def obs_message_remove_expired(full_time):
    expired_epochs = [ t for t in obs_messages.keys() if t <= full_time ]
    for t in expired_epochs:
        del obs_messages[t]

# 'msg' was just received from an input stream, check if it completes a sequence
def multiplex(msg):
    global last_sent_time

    if msg.msg_type == sbp.observation.SBP_MSG_OBS:
        # we need to fully decode the message if not just forwarding it
        msg = dispatch(msg)
        full_time = get_full_time(msg)
        if last_sent_time is None or full_time > last_sent_time: # not interested in MSG_OBS messages older than last sent message sequence
            obs_message_add(msg)
            # check if we now have a complete sequence as a result of adding the message
            msg_sequence = obs_message_get_sequence(full_time)
            if msg_sequence is not None:
                send_messages_via_udp(msg_sequence)
                obs_message_remove_expired(full_time)
                last_sent_time = full_time
    else:
        # not MSG_OBS, forward immediately
        msg.sender = 0 # overwrite sender ID
        send_messages_via_udp([msg])

def send_messages_via_udp(msgs):
    for msg in msgs:
        if msg.msg_type == sbp.observation.SBP_MSG_OBS:
            sender = get_sender(msg)
            rospy.loginfo(sender + ", " + str(msg.header.t.tow) + ", " + str(msg.header.n_obs))
        msg.sender = 0 # overwrite sender ID
        udp.call(msg) # udp logger packs the msgs to binary before sending

def get_sender(msg):
    if msg.sender == ntrip_sender:
        sender = "Ntrip"
    elif msg.sender == radio_sender:
        sender = "Radio"
    else:
        sender = str(msg.sender)
        rospy.logwarn("Wrong sender definition: " + sender)
    return sender

def ntrip_corrections(q_ntrip):
    # run command to listen to ntrip client, convert from rtcm3 to sbp and from sbp to json redirecting the stdout
    str2str_cmd = ["str2str", "-in", "ntrip://{}:{}/{}".format(NTRIP_HOST, NTRIP_PORT, NTRIP_MOUNT_POINT)]
    rtcm3tosbp_cmd = ["rtcm3tosbp", "-d", "{}:{}".format(*get_current_time())]
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

        if sbp_msg.msg_type == sbp.observation.SBP_MSG_OBS and ntrip_sender is None:
            global ntrip_sender
            ntrip_sender = sbp_msg.sender # get ntrip sender id

        q_ntrip.put(sbp_msg)


def radio_corrections(q_radio):
    sbp_msg_types = [SBP_MSG_OBS, SBP_MSG_GLO_BIASES, SBP_MSG_BASE_POS_ECEF, SBP_MSG_EPHEMERIS_BDS, SBP_MSG_EPHEMERIS_GAL, SBP_MSG_EPHEMERIS_GLO, SBP_MSG_EPHEMERIS_QZSS]
    global radio_sender # get radio sender id
    with PySerialDriver(RADIO_PORT, baud=RADIO_BAUDRATE) as driver:
        print(driver.read)
        with Handler(Framer(driver.read, None, verbose=False)) as source:
            try:
                for sbp_msg, metadata in source.filter(sbp_msg_types):
                    if sbp_msg.msg_type == sbp.observation.SBP_MSG_OBS and radio_sender is None:
                        radio_sender = sbp_msg.sender
                    q_radio.put(sbp_msg)

            except KeyboardInterrupt:
                pass

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

    ntrip_msgs = []
    radio_msgs = []

    # Arbitrate
    while not rospy.is_shutdown():
        while len(ntrip_msgs)==0 and len(radio_msgs)==0:
            ntrip_msgs = get_queue_msgs(q_ntrip)
            radio_msgs = get_queue_msgs(q_radio)

        for msg in ntrip_msgs + radio_msgs:
                multiplex(msg)
        ntrip_msgs = []
        radio_msgs = []
