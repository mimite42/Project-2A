#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
import actionlib
import test.msg

class RobotActionServer:
    def __init__(self, name):
        self._action_name = name
        self.move_group = MoveGroupCommander("ur_robot")
        self.move_group_gripper = MoveGroupCommander("gripper")
        self.server = actionlib.SimpleActionServer(
            self._action_name,
            test.msg.RobotAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("Server Ready...")

    def move_to_position(self, x, y, z):
        initial_joint_values = self.move_group.get_current_joint_values()

        pose_target = PoseStamped()
        pose_target.header.frame_id = "base_link"
        pose_target.pose.position.x = x
        pose_target.pose.position.y = y
        pose_target.pose.position.z = 0.1
        pose_target.pose.orientation.w = 1

        # Assuming no rotation
        current_orientation = self.move_group.get_current_pose().pose.orientation
        pose_target.pose.orientation = current_orientation

        self.move_group.set_pose_target(pose_target)
        self.move_group.set_planning_time(5.0)  # Set planning time in seconds
        self.move_group.set_num_planning_attempts(5)  # Set number of planning attempts
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Assuming "gripper" is your gripper planning group
        gripper_values_open = [0.0, 0.0]  # Joint values for the gripper open state
        gripper_values_closed = [0.018, 0.018]  # Joint values for the gripper closed state

        # Close gripper
        self.move_group_gripper.set_joint_value_target(gripper_values_closed)
        self.move_group_gripper.set_planning_time(5.0)  # Set planning time in seconds
        self.move_group_gripper.set_num_planning_attempts(5)  # Set number of planning attempts
        plan = self.move_group_gripper.go(wait=True)
        self.move_group_gripper.stop()
        self.move_group_gripper.clear_pose_targets()

        rospy.sleep(4.5)  # Simulate gripper closing time

        # Set the robot to the same pose but with a modified y coordinate
        pose_target.pose.position.y = -y  # Modify the y coordinate
        self.move_group.set_pose_target(pose_target)
        self.move_group.set_planning_time(5.0)  # Set planning time in seconds
        self.move_group.set_num_planning_attempts(5)  # Set number of planning attempts
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Open gripper
        self.move_group_gripper.set_joint_value_target(gripper_values_open)
        self.move_group_gripper.set_planning_time(5.0)  # Set planning time in seconds
        self.move_group_gripper.set_num_planning_attempts(5)  # Set number of planning attempts
        plan = self.move_group_gripper.go(wait=True)
        self.move_group_gripper.stop()
        self.move_group_gripper.clear_pose_targets()

    def execute_cb(self, goal):
        rospy.loginfo("Received goal: %s", goal)

        rows = goal.rows
        cols = goal.cols

        for i in range(rows):
            for j in range(cols):
                marker_id = i * cols + j
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "my_namespace"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = goal.first_x + j * 0.05
                marker.pose.position.y = goal.first_y - i * 0.035
                marker.pose.position.z = goal.first_z + 0.5
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                # Move the robot to the marker position
                self.move_to_position(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)

                # Simulate processing time
                rospy.sleep(2.0)

        rospy.loginfo("Action execution completed.")

if __name__ == '__main__':
    rospy.init_node("Robot")
    server = RobotActionServer(rospy.get_name())
    rospy.spin()

