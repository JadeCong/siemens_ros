#!/usr/bin/env python

import sys, os, time
import numpy as np

import rospy
from dynamic_reconfigure.server import Server
from siemens_plc.cfg import laser_config_paramConfig
from siemens_plc.siemens_plc_client import SiemensPlcClient
from std_msgs.msg import Int32MultiArray as HoldingRegister
from std_msgs.msg import Bool


# define the laser config parameters: laser_config_1, laser_config_2, laser_config_3
laser_config_1, laser_config_2, laser_config_3 = None, None, None


def laser_config_param_callback(config, level):
    # declare the global variables
    global laser_config_1, laser_config_2, laser_config_3
    
    # set the global variables for laser config
    laser_config_1 = config.laser_config_1
    laser_config_2 = config.laser_config_2
    laser_config_3 = config.laser_config_3
    
    return config

def check_laser_switch_callback(msg, args):
    # get the laser_switch msgs and make laser_config array
    args[1].data = [laser_config_1, laser_config_2, laser_config_3, msg]
    
    # publish the laser config parameters
    args[0].publish(args[1])

def siemens_plc_interface_node():
    # declare the global variables
    global laser_config_1, laser_config_2, laser_config_3
    
    # init ros node
    rospy.init_node("siemens_plc_interface", anonymous=True)
    
    # get ros parameters from parameter server
    host = rospy.get_param("/siemens/siemens_plc_interface/host")
    port = rospy.get_param("/siemens/siemens_plc_interface/port")
    rate = rospy.get_param("/siemens/siemens_plc_interface/rate")
    reset_registers = rospy.get_param("/siemens/siemens_plc_interface/reset_registers")
    sub_topic = rospy.get_param("/siemens/siemens_plc_interface/sub_topic")
    pub_topic = rospy.get_param("/siemens/siemens_plc_interface/pub_topic")
    topic_laser_switch = rospy.get_param("/siemens/siemens_plc_interface/topic_laser_switch")
    
    # start the dynamic reconfigure parameter server
    dynamic_reconfigure_parameter_server = Server(laser_config_paramConfig, laser_config_param_callback)
    
    # setup modbus client for Siemens PLC
    plc_client = SiemensPlcClient(host, port, rate, reset_registers, sub_topic, pub_topic)
    
    # start listening to modbus
    plc_client.startListening()
    
    # define the publisher for writing laser config parameters to modbus registers.
    pub_laser_config = rospy.Publisher(sub_topic, HoldingRegister,
                                        queue_size=1,
                                        tcp_nodelay=True,
                                        latch=False)
    laser_config = HoldingRegister()
    
    # define the subscriber for getting the status of laser switch
    sub_laser_switch = rospy.Subscriber(topic_laser_switch, Bool,
                                        callback=check_laser_switch_callback, 
                                        callback_args=[pub_laser_config, laser_config], 
                                        queue_size=1, 
                                        tcp_nodelay=True)
    
    # TODO: Bugs: no rospy.spinOnce for controlling the frequency of receiving msgs from master_hfd to match PLC server rate, so set the quene_size of publisher/subscriber as 1 to discard the redundant msgs.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    # stop the listener on the modbus
    plc_client.stopListening()


if __name__ == "__main__":
    # run ros node
    siemens_plc_interface_node()
