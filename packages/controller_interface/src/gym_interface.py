#!/usr/bin/env python3

import rospy
import json
import yaml
import time
import os.path

from duckietown_msgs.msg import WheelsCmdStamped
from std_srvs.srv import EmptyResponse

from duckietown.dtros import DTROS, NodeType, TopicType
from controller_interface_node import ControllerInterface

# This is the importent part:
from your_AI_file import your_AI

dt_town=ControllerInterface("controller_interface_node",your_AI)

#run at 30 Hz
rostime.run(30, dt_town.car_cmd_call()) # I forget the exact functions
rospy.spin()