#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import rospy

HOST = "localhost"
PORT = 4223
UID_idr = "KqZ"

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_industrial_dual_relay import BrickletIndustrialDualRelay


# Turns off industrial dual relay on Ctrl-C
def turn_off():
    idr.set_selected_value(1, False)
    rospy.loginfo("Tool turned off.")
    ipcon.disconnect()


# To turn on whatever tool is attached to the tool changer.
if __name__ == "__main__":
    ipcon = IPConnection()
    idr = BrickletIndustrialDualRelay(UID_idr, ipcon)
    rospy.init_node('turn_on_tools', anonymous=True)
    rospy.on_shutdown(turn_off)

    ipcon.connect(HOST, PORT)
    
    idr.set_selected_value(1, True)
    rospy.loginfo("Tool turned on.")

    rospy.spin()
