import copy
import sys

import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown


class Arm(object):
    def __init__(self, move_group):
        # create move group
        self.group = MoveGroupCommander(move_group)

    def get_pose(self):
        """
        Get current pose for specified move group

        Returns
            ros msg (geometry_msgs.msg._PoseStamped.PoseStamped) - A message
                specifying the pose of the move group as position (XYZ) and
                orientation (xyzw).
        """
        return self.group.get_current_pose().pose

    def get_joint_values(self):
        """
        Get joint values for specified move group

        Returns
            joint value (list) - A list of joint values in radians
        """
        return self.group.get_current_joint_values()

    def get_planning_frame(self):
        """
        Get planning frame of move group
        """
        return self.group.get_planning_frame()

    def clear_pose_targets(self, move_group):
        """
        Clear target pose for planning
        """
        self.group.clear_pose_targets()

    def plan_to_pose(self, pose):
        """
        Plan to a given pose for the end-effector

        Args
            pose (msg): geometry_msgs.msg.Pose
        """
        # Clear old pose targets
        self.group.clear_pose_targets()

        # Set target pose
        self.group.set_pose_target(pose)

        numTries = 0;
        while numTries < 5:
            plan = self.group.plan()
            numTries+=1
            if len(plan.joint_trajectory.points) > 0:
                print('succeeded in %d tries' % numTries)
                return True
        print('Planning failed')
        return False

    def plan_to_joints(self, joint_angles):
        """Plan to a given joint configuration

        Args
            joint_angles (list of floats): list of len 7 of joint angles in radians
        Returns
            True if planning succeeds, False if not
        """
        # Clear old pose targets
        self.group.clear_pose_targets()

        # Set Joint configuration target
        self.group.set_joint_value_target(joint_angles)
        numTries = 0
        while numTries < 5:
            plan = group.plan()
            numTries+=1
            if len(plan.joint_trajectory.points) > 0:
                print('succeeded in %d tries' % numTries)
                return True
        print("Planning failed")
        return False

    def move_to_joints(self, joint_angles):
        """
        Move to specified joint configuration

        Args
            joint_angles (list of floats): list of len 7 of joint angles in radians
            blocking (bool): If True,
        Returns
            True if planning succeeds, False if not
        """
        # plan
        plan = self.plan_to_joints(group.get_name(), joint_angles)

        # move to joint configuration
        if plan:
            group.go(wait=True)
        return plan

    def move_to_pose(self, pose):
        """
        Move to specified joint configuration

        Args
            pose (msg): geometry_msgs.msg.Pose
            joint_angles (list of floats): list of len 7 of joint angles in radians
            blocking (bool): If True,
        Returns
            True if planning succeeds, False if not
        """
        # plan
        plan = self.plan_to_pose(pose)

        if plan:
            group.go(wait=blocking)
        group.stop()
        group.clear_pose_targets()
        return

    def _change_planner(self, planner):
        planners = [
            "SBLkConfigDefault",
            "ESTkConfigDefault",
            "LBKPIECEkConfigDefault",
            "BKPIECEkConfigDefault",
            "KPIECEkConfigDefault",
            "RRTkConfigDefault",
            "RRTConnectkConfigDefault",
            "RRTstarkConfigDefault",
            "TRRTkConfigDefault",
            "PRMkConfigDefault",
            "PRMstarkConfigDefault"
        ]
        if planner in planners:
            self.group.set_planner_id(planner)
        return

    def create_subscriber(self, topic, message_type):
        return
