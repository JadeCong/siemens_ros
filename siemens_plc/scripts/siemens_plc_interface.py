#!/usr/bin/env python3
# -- coding: utf-8 --

import sys, os, time
import numpy as np
import struct
import math

import rospy
from dynamic_reconfigure.server import Server
from siemens_plc.cfg import laser_config_paramConfig
from siemens_plc.siemens_plc_client import SiemensPlcClient
from std_msgs.msg import Int32MultiArray as HoldingRegister


# define the laser config parameters
ready_flag, powder_feed_start, blow_gas_start, emit_laser_start, laser_power, powder_feed_rate = None, None, None, None, None, None


def laser_config_param_callback(config, level):
    # declare the global variables
    global ready_flag, powder_feed_start, blow_gas_start, emit_laser_start, laser_power, powder_feed_rate
    
    # set the global variables for laser config
    ready_flag = config.ready_flag
    powder_feed_start = config.powder_feed_start
    blow_gas_start = config.blow_gas_start
    emit_laser_start = config.emit_laser_start
    laser_power = config.laser_power
    powder_feed_rate = config.powder_feed_rate
    
    return config

def laser_status_callback(msg, args):
    # declare the global variables
    global ready_flag, powder_feed_start, blow_gas_start, emit_laser_start, laser_power, powder_feed_rate
    
    # get the laser_status msgs and update the laser_config array
    args[2].update_configuration({"heart_beat": str(msg.data[0])})
    args[1].data = [msg.data[0], ready_flag, powder_feed_start, blow_gas_start, emit_laser_start, laser_power, powder_feed_rate]
    
    # publish the laser config parameters
    args[0].publish(args[1])

def siemens_plc_interface_node():
    # declare the global variables
    global ready_flag, powder_feed_start, blow_gas_start, emit_laser_start, laser_power, powder_feed_rate
    
    # init ros node
    rospy.init_node("siemens_plc_interface", anonymous=True)
    
    # get ros parameters from parameter server
    host = rospy.get_param("~host")
    port = rospy.get_param("~port")
    rate = rospy.get_param("~rate")
    reset_registers = rospy.get_param("~reset_registers")
    sub_topic = rospy.get_param("~sub_topic")
    pub_topic = rospy.get_param("~pub_topic")
    
    # start the dynamic reconfigure parameter server
    dynamic_reconfigure_parameter_server = Server(laser_config_paramConfig, laser_config_param_callback)
    
    # setup modbus client for Siemens PLC
    plc_client = SiemensPlcClient(host, port, rate, reset_registers, sub_topic, pub_topic)
    
    # start listening to modbus
    plc_client.startListening()
    rospy.loginfo("Modbus listener started.")
    
    # define the publisher for writing the laser config parameters to modbus registers
    pub_laser_config = rospy.Publisher(sub_topic, HoldingRegister,
                                        queue_size=1,
                                        tcp_nodelay=True,
                                        latch=False)
    laser_config = HoldingRegister()
    
    # define the subscriber for reading the status of laser config from modbus registers
    sub_laser_status = rospy.Subscriber(pub_topic, HoldingRegister,
                                        callback=laser_status_callback,
                                        callback_args=[pub_laser_config, laser_config, dynamic_reconfigure_parameter_server],
                                        queue_size=1,
                                        tcp_nodelay=True)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    # stop the listener on the modbus and close connection
    plc_client.stopListening()
    plc_client.closeConnection()


if __name__ == "__main__":
    # run ros node
    siemens_plc_interface_node()
