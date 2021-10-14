#!/usr/bin/env python

import rospy
import os
import datetime
import subprocess
import copy
import sys
import struct
import time
import operator
import threading
import Queue as queue
import argparse

# NOTE: the dependency on MAVProxy can be avoided by making a local copy of
# https://github.com/ArduPilot/MAVProxy/blob/master/MAVProxy/modules/lib/rtcm3.py
import rtcm3
import ntrip
from multiprocessing import Process, Value
from multiprocessing.managers import BaseManager


# Represent a parsed RTCM3 message
class RTCM3_Message(rtcm3.RTCM3, object):
    def __init__(self):
        super(RTCM3_Message,self).__init__(debug=False)


    def get_tow(self):
        msg_id = self.get_packet_ID()
        assert(msg_id is not None and msg_id in RTCM3_OBS)

        if self.parsed_pkt is None or len(self.parsed_pkt) < 10:
            return None

        tow_ms, = struct.unpack('>L', self.parsed_pkt[6:10])
        tow_ms >>= 2

        if msg_id in RTCM3_OBS_GPS: # DF004
            return tow_ms
        elif msg_id in RTCM3_OBS_GLO: # DF416/DF034
            day_of_week = tow_ms >> 27
            tod_ms = tow_ms & 0x7ffffff
            # FIXME: don't automatically hard code number of leap seconds,
            # determine automatically from ephemerides instead
            tow_ms = day_of_week * MS_PER_DAY + tod_ms - UTC_SU_OFFSET * 3600 * 1000 + NUM_LEAP_SECONDS * 1000
            while tow_ms < 0:
                tow_ms += MS_PER_WEEK
            return tow_ms
        elif msg_id in RTCM3_OBS_GAL: # DF248
            return tow_ms
        elif msg_id in RTCM3_OBS_BDS: # DF+002
            # handle underflow
            if tow_ms >= (2 ** 30) - BDS_SECOND_TO_GPS_SECOND * 1000:
                tow_ms = RTCM_MAX_TOW_MS + 1 - (2 ** 30) - tow_ms
            # BDS system time has a constant offset
            tow_ms += BDS_SECOND_TO_GPS_SECOND * 1000
            if tow_ms >= MS_PER_WEEK:
                tow_ms -= MS_PER_WEEK
            return tow_ms
        elif msg_id in RTCM3_OBS_QZS: # DF428
            return tow_ms


class RTCM3_Parser():
    def __init__(self):
        self.msg = RTCM3_Message()

    # feed each byte in 'data' into the parser and yield an RTCM3_Message
    # upon success
    def parse(self, data):
        for byte in data:
            if self.msg.read(chr(byte)):
                yield copy.copy(self.msg)


# List of messages required to form a full epoch.
# NOTE: this approach means that the first couple of epochs may only include
# partial message sets (until at least one of each expected type has
# been successfully received).  This list can be pre-populated with with the
# expected message types in order to work around this issue.
expected_messages = []

# Dictionary of lists containing RTCM_Messagess where we have not yet
# received a full message sequence.  The key is a Time Of Week value
# which represents the number of milliseconds since the beginning of
# the week.
obs_messages = {}

# TOW of last message sequence sent via UDP
last_sent_time = None

# handle MSM4, MSM5, MSM6 or MSM7 messages
RTCM3_OBS_GPS = [ 1074, 1075, 1076, 1077 ]
RTCM3_OBS_GLO = [ 1084, 1085, 1086, 1087 ]
RTCM3_OBS_GAL = [ 1094, 1095, 1096, 1097 ]
RTCM3_OBS_BDS = [ 1124, 1125, 1126, 1127 ]
RTCM3_OBS_QZS = [ 1114, 1115, 1116, 1117 ]

RTCM3_OBS = RTCM3_OBS_GPS + RTCM3_OBS_GLO + RTCM3_OBS_GAL + RTCM3_OBS_BDS + RTCM3_OBS_QZS

# Constant difference between BDS time and GPS time
BDS_SECOND_TO_GPS_SECOND = 14
# UTC (SU) offset (hours)
UTC_SU_OFFSET = 3
# milliseconds per day
MS_PER_DAY = 24 * 3600 * 1000
# milliseconds per week
MS_PER_WEEK = 7 * 24 * 3600 * 1000
# maximum value for time-of-week in integer milliseconds
RTCM_MAX_TOW_MS = MS_PER_WEEK - 1
# number of leap seconds between UTC and GPS time
NUM_LEAP_SECONDS = 18

# NTRIP host
NTRIP_HOST = rospy.get_param('/rtcm3_arbitrator/ntrip_host', "rtk2go.com")
NTRIP_PORT = rospy.get_param('/rtcm3_arbitrator/ntrip_port', 2101)
NTRIP_MOUNT_POINT = rospy.get_param('/rtcm3_arbitrator/ntrip_mount_point', "ER_Valldoreix_1")
#RADIO
RADIO_PORT =  rospy.get_param('/rtcm3_arbitrator/radio_port', "/dev/freewaveGXMT14")
RADIO_BAUDRATE = rospy.get_param('/rtcm3_arbitrator/radio_baudrate', 115200)
# UDP LOGGER
UDP_ADDRESS = rospy.get_param('/rtcm3_arbitrator/udp_address', "192.168.8.222")
UDP_PORT =  rospy.get_param('/rtcm3_arbitrator/udp_port', 55558)

freq = rospy.get_param('/rtcm3_arbitrator/frequency', 5)
debug = rospy.get_param('/rtcm3_arbitrator/debug', False)

# create instance of UdpLogger object
#udp = UdpLogger(UDP_ADDRESS, UDP_PORT)

# txt files for msg logging
if debug:
    HOME = os.getenv('HOME')
    now = datetime.datetime.utcnow()
    txt_time = now.strftime("%Y%m%d%H%M%S")
    ntrip_file_path = HOME + "/sbp_arb_logs/" + txt_time + "_ntrip.txt"
    radio_file_path = HOME + "/sbp_arb_logs/" + txt_time + "_radio.txt"
    arb_file_path = HOME + "/sbp_arb_logs/" + txt_time + "_arbitrator.txt"
    ntrip_txt_file = open(ntrip_file_path, "a")
    radio_txt_file = open(radio_file_path, "a")
    arb_txt_file = open(arb_file_path, "a")


# get current year:month:day:hour
# def get_current_time():
#    now = datetime.datetime.utcnow()
#    return "{}:{}:{}:{}".format(now.year, now.month, now.day, now.hour)


# Return True if 'tow_ms_new' is after 'tow_ms_prev', accounting for potential
# end of week wrapping

# provisional
def get_toww(msg, msg_id):

    if msg is None or len(msg) < 10:
        return None

    tow_ms, = struct.unpack('>L', msg[6:10])
    print("Tow 32 bits: " + str(tow_ms))
    tow_ms >>= 2
    print("Tow 30 bits: " + str(tow_ms))

    if msg_id in RTCM3_OBS_GPS: # DF004
        return tow_ms
    elif msg_id in RTCM3_OBS_GLO: # DF416/DF034
        day_of_week = tow_ms >> 27
        print("Day of week GLO: " + str(day_of_week))
        tod_ms = tow_ms & 0x7ffffff
        # FIXME: don't automatically hard code number of leap seconds,
        # determine automatically from ephemerides instead
        tow_ms = 1 * MS_PER_DAY + tod_ms - UTC_SU_OFFSET * 3600 * 1000 + NUM_LEAP_SECONDS * 1000
        while tow_ms < 0:
            tow_ms += MS_PER_WEEK
        return tow_ms
    elif msg_id in RTCM3_OBS_GAL: # DF248
        return tow_ms
    elif msg_id in RTCM3_OBS_BDS: # DF+002
        # handle underflow
        if tow_ms >= (2 ** 30) - BDS_SECOND_TO_GPS_SECOND * 1000:
            tow_ms = RTCM_MAX_TOW_MS + 1 - (2 ** 30) - tow_ms
        # BDS system time has a constant offset
        tow_ms += BDS_SECOND_TO_GPS_SECOND * 1000
        if tow_ms >= MS_PER_WEEK:
            tow_ms -= MS_PER_WEEK
        return tow_ms
    elif msg_id in RTCM3_OBS_QZS: # DF428
        return tow_ms


def is_tow_greater_than(tow_ms_prev, tow_ms_new):
    if tow_ms_new < MS_PER_DAY and tow_ms_prev > 6 * MS_PER_DAY:
        return True
    return True if tow_ms_new > tow_ms_prev else False


# try and add 'msg' (of type RTCM3_Message) to obs_messages
def obs_message_add(msg):
    msg_id = msg.get_packet_ID()
    assert(msg_id is not None and msg_id in RTCM3_OBS)
    tow_ms = msg.get_tow()

    if tow_ms in obs_messages:
        # check for existing message with same message ID
        matching_msgs = [ m for m in obs_messages[tow_ms] if m.get_packet_ID() == msg_id ]
        if len(matching_msgs) > 0:
            if matching_msgs[0].get_packet() != msg.get_packet():
                print("Sanity check failed on RTCM message with tow {}, msg_id {}".format(tow_ms, msg_id))
                return
        else:
            # no existing message, so add to list and resort based on message ID
            obs_messages[tow_ms].append(msg)
            obs_messages[tow_ms].sort(key=lambda m: m.get_packet_ID())
    else:
        # no existing messages for this epoch
        obs_messages[tow_ms] = [msg]


# Check if we have a complete sequence for 'tow_ms' in obs_messages.
# If so, return the list of RTCM3_Message objects.
def obs_message_get_sequence(tow_ms):
    if tow_ms not in obs_messages:
        return None

    msgs = obs_messages[tow_ms]

    # check that all expected messages are present
    msg_ids = [ m.get_packet_ID() for m in msgs ]
    return msgs if msg_ids == expected_messages else None


# Remove epochs with times equal to or older than 'tow_ms' from obs_messages
def obs_message_remove_expired(tow_ms):
    expired_epochs = [ t for t in obs_messages.keys() if not is_tow_greater_than(tow_ms, t) ]
    for t in expired_epochs:
        del obs_messages[t]


# for testing purposes, just print to stdout
def send_messages_via_udp(msgs):
    for msg in msgs:
        sys.stdout.buffer.write(msg.get_packet())


# 'msg' was just received from an input stream, check if it completes a sequence
def multiplex(msg):
    global last_sent_time

    msg_id = msg.get_packet_ID()

    if msg_id in RTCM3_OBS: # observation message
        tow_ms = msg.get_tow()

        if msg_id not in expected_messages:
            expected_messages.append(msg_id)
            expected_messages.sort()
        if last_sent_time is None or is_tow_greater_than(last_sent_time, tow_ms): # not interested in obs messages older than last sent message sequence
            obs_message_add(msg)
            # check if we now have a complete sequence as a result of adding the message
            msg_sequence = obs_message_get_sequence(tow_ms)
            if msg_sequence is not None:
                send_messages_via_udp(msg_sequence)
                obs_message_remove_expired(tow_ms)
                last_sent_time = tow_ms
    else: # not an observation message, forward immediately
        send_messages_via_udp([msg])


### EVERYTHING BELOW THIS POINT IS FOR TESTING PURPOSES ONLY ###
# Read RTCM file in 500 bytes chunks.
# Randomly drop messages from "VHF" and "Cellular" streams, attempt to
# parse each stream as RTCM3, then pass any decoded messages to multiplex().

# parser_vhf = RTCM3_Parser()
# parser_cellular = RTCM3_Parser()

# def create_simulated_streams(data):
#     # randomly drop 0 to 2 bytes from 'stream'
#     def random_drop(stream):
#         for i in range(0, random.randint(0, 2)):
#             if len(stream) == 0:
#                 return
#             randpos = random.randint(0, len(stream)-1)
#             stream = stream[:randpos] + stream[randpos + 1:]
#         return stream
#
#     # parse messages from VHF stream
#     for msg in parser_vhf.parse(random_drop(data)):
#         multiplex(msg)
#     # parse messages from cellular stream
#     for msg in parser_cellular.parse(random_drop(data)):
#         multiplex(msg)
#
#
# def test(filename):
#     with open(filename, "rb") as infile:
#         while True:
#             b = infile.read(500)
#             if len(b) == 0:
#                 break
#             create_simulated_streams(b)

#######################################################################

#get msgs from queue object
def get_queue_msgs(queue):
    msg_list = []
    while not queue.empty():
         msg_list.append(queue.get())
    return msg_list


def get_sender(msg):
    #print("Ntrip sender: " + str(ntrip_sender) + ", msg sender: " +str(msg.sender))
    if msg.sender == ntrip_sender:
        sender = "Ntrip"
    elif msg.sender == radio_sender:
        sender = "Radio"
    else:
        sender = str(msg.sender)
        rospy.logwarn("Wrong sender definition: " + sender)
    return sender

def get_sender_id(msg):
    sender_id, = struct.unpack('>H', data[4:6])
    sender_id &= 0xfff
    return sender_id

def ntrip_corrections(q_ntrip):
    # global ntrip_sender
    ntrip_rate = rospy.Rate(10) # 10hz
    ntrip_client = ntrip.NtripClient(port = NTRIP_PORT, caster = NTRIP_HOST, mountpoint = NTRIP_MOUNT_POINT)

    while not rospy.is_shutdown():
        data = ntrip_client.read()
        rtcm_id = ntrip_client.get_ID()
        if data is None:
            #print("Data is None")
            continue

        print("Msg type: " + str(rtcm_id) + ", got: ", len(data))

        if rtcm_id in RTCM3_OBS:
            tow_ms = get_toww(data, rtcm_id)
            print(tow_ms)

        ntrip_rate.sleep()
        #except KeyboardInterrupt:
                #pass


def radio_corrections(q_radio):
    global radio_sender # get radio sender id
    radio_rate = 5 #hz
    with PySerialDriver(RADIO_PORT, baud=RADIO_BAUDRATE) as driver:
        print(driver.read)
        with Handler(Framer(driver.read, None, verbose=False)) as source:
            try:
                for sbp_msg, metadata in source.filter():

                    start_time = rospy.get_time()

                    if radio_sender is None:
                        radio_sender = sbp_msg.sender

                    q_radio.put(sbp_msg)

                    if debug and sbp_msg.msg_type == sbp.observation.SBP_MSG_OBS:
                        radio_txt_file.write(str(dispatch(sbp_msg)) + "\n")

                    end_time = rospy.get_time()
                    sleep_time = max([0, 1/radio_rate - (end_time - start_time)])
                    rospy.sleep(sleep_time)

            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    rospy.init_node('sbp_arbitrator', anonymous=True)

    q_ntrip = queue.Queue()
    q_radio = queue.Queue()

    th1 = threading.Thread(target=ntrip_corrections,args=(q_ntrip,))
    #th2 = threading.Thread(target=radio_corrections,args=(q_radio,))

    th1.start()
    #th2.start()

    #ntrip_data = []

    #parser_cellular = RTCM3_Parser()

    #main_rate = rospy.Rate(freq) # 10hz

    # Arbitrate
    # while not rospy.is_shutdown():
    #     while len(ntrip_data)==0:
    #         ntrip_data = get_queue_msgs(q_ntrip)
    #         #radio_msgs = get_queue_msgs(q_radio)
    #
    #     print(ntrip_data)
    #     print("Hello")
    #
    #     for msg in parser_cellular.parse(ntrip_data):
    #         multiplex(msg)
    #
    #     ntrip_data = []
    #
    #     main_rate.sleep()
