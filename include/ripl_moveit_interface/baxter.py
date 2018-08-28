import copy
import sys

import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown
from baxter_interface import CHECK_VERSION, Gripper

from arm import Arm

class Baxter(object):
    def __init__(self):
        # create a RobotCommander
        self.robot = RobotCommander()

        # create a PlanningScene Interface object
        self.scene = PlanningSceneInterface()

        # create left arm move group and gripper
        self.left_arm = Arm("left_arm")
        self.left_gripper = self.left_arm.gripper = Gripper("left")

        # create right arm move group and gripper
        self.right_arm = Arm("right_arm")
        self.right_gripper = self.right_arm.gripper = Gripper("right")

        self.gripper_offset = 0.0

        # reset robot
        self.reset()

    def reset(self, ):
        """
        Reset robot state and calibrate grippers
        """
        # enable robot
        self.enable()

        # move to ready position
        joint_angles = [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
        self.move_to_joints("right_arm", joint_angles, blocking=False)
        self.move_to_joints("left_arm", joint_angles, blocking=True)

        # calibrate grippers
        self.calibrate_grippers()
        return

    def enable(self):
        """
        Enable robot
        """
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()

    def calibrate_grippers(self):
        """
        Calibrate grippers
        """
        if not self.left_gripper.calibrated():
            self.left_gripper.calibrate()
        if not self.right_gripper.calibrated():
            self.right_gripper.calibrate()
        return

    def get_robot_state(self):
        """
        Get robot state

        Returns
            ros msg (moveit_msgs.msg._RobotState.RobotState) - A message that contains
                information about all of the joints, including fixed head pan,
                revolute arm joints, and prismatic finger joints
        """
        return self.robot.get_current_state()

    def get_group_names(self):
        """
        Get group names

        Returns
            group names (list): a list of move groups that can be used for planning
        """
        return self.robot.get_group_names()

    def pick(self):
        return

    def place(self):
        return
