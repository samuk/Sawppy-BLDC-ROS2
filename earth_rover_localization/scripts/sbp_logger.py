#!/usr/bin/env python

#https://stackoverflow.com/questions/34337514/updated-variable-into-multiprocessing-python
import rospy
import os
import datetime
import subprocess
import json
import sbp.msg
import sys
import time

from threading import Thread
from multiprocessing import Process, Value
from sbp.client.loggers.udp_logger import UdpLogger
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.observation import SBP_MSG_OBS, MsgObs, SBP_MSG_GLO_BIASES, MsgGloBiases, SBP_MSG_BASE_POS_ECEF, MsgBasePosECEF
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
#def get_current_time():
#    now = datetime.datetime.now(datetime.timezone.utc)
#    return "{}:{}:{}:{}".format(now.year, now.month, now.day, now.hour)

def ntrip_corrections():
    ntrip_epoch = None
    ntrip_tow = 0

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
        if sbp_general_msg.msg_type == 72:
            sbp_msg = MsgBasePosECEF(sbp_general_msg)
        if sbp_general_msg.msg_type == 74:
            sbp_msg = MsgObs(sbp_general_msg)
            ntrip_epoch = ntrip_tow = sbp_msg.header.t.tow # update epoch
            rospy.loginfo("Ntrip, %i, %i, %s", ntrip_tow, sbp_msg.msg_type, hex(sbp_msg.header.n_obs))
        if sbp_general_msg.msg_type == 117:
            sbp_msg = MsgGloBiases(sbp_general_msg)
            rospy.loginfo("Ntrip, %i, %i, None", ntrip_tow, sbp_msg.msg_type)

        udp.call(sbp_msg)

def radio_corrections():
    last_radio_epoch = None
    radio_epoch = None
    radio_tow = 0

    with PySerialDriver(RADIO_PORT, baud=RADIO_BAUDRATE) as driver:
        print(driver.read)
        with Handler(Framer(driver.read, None, verbose=False)) as source:
            try:
                for msg, metadata in source.filter([SBP_MSG_OBS, SBP_MSG_GLO_BIASES, SBP_MSG_BASE_POS_ECEF]):
                    # change radio sender ID to ntrip to avoid temporal glitch
                    #msg.sender = 65202
                    # update epoch
                    if msg.msg_type == 74:
                        radio_epoch = radio_tow = msg.header.t.tow
                        rospy.loginfo("Radio, %i, %i, %s", radio_tow, msg.msg_type, hex(msg.header.n_obs))
                    if msg.msg_type == 117:
                        rospy.loginfo("Radio, %i, %i, None", radio_tow, msg.msg_type)

                    udp.call(msg)
                    
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    rospy.init_node('sbp_arbitrator', anonymous=True)
    p1 = Process(target=ntrip_corrections)
    p1.start()
    p2 = Process(target=radio_corrections)
    p2.start()
    p1.join()
    p2.join()
