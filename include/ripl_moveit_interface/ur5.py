import rospy
import numpy as np
import signal
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown
import moveit_msgs.msg
import geometry_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from model_fitting.srv import segment
from enum import Enum

import sys
from time import sleep

from arm import Arm
from gripper import Gripper

class UR5(object):
    def __init__(self):
        # create a RobotCommander
        self.robot = RobotCommander()

        # create a PlanningSceneInterface object
        self.scene = PlanningSceneInterface()

        # create arm
        self.arm = Arm("manipulator")

        # create gripper
        self.gripper = Gripper()
