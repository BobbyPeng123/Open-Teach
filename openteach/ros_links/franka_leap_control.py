import rospy
import numpy as np
import time
from LEAP_Hand_API.ros_module.LeapController import LeapController
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from kinova_arm.controller import KinovaController
from copy import deepcopy as copy
import math
sys.path.append(os.path.abspath('/home/kovaak/leap_hand_teleoperator/LEAP_Hand_API/python'))
from LeapController import LeapNode
PORT = 'ttyUSB0'
BAUDRATE = 4000000

class DexArmControl():
    def __init__(self,record_type=None, robot_type='kinova'):
        try:
            rospy.init_node("dex_arm", disable_signals = True, anonymous = True)
        except:
            pass

        if robot_type == 'both':
            self._init_leap_hand_control()
            self._init_franka_arm_control()
        elif robot_type == 'leap':
            self._init_leap_hand_control()
        elif robot_type == 'kinova':
            self._init_franka_arm_control()


    def _init_leap_hand_control(self):
        self.leap = LeapNode(PORT,BAUDRATE)