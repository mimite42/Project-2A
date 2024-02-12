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
        self.move_group = MoveGroupCommander("arm")
        
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
        pose_target.header.frame_id = "world"
        pose_target.pose.position.x = x
        pose_target.pose.position.y = y
        pose_target.pose.position.z = z

        # Assuming no rotation
        current_orientation = self.move_group.get_current_pose().pose.orientation
        current_position = self.move_group.get_current_pose().pose.position
        pose_target.pose.orientation = current_orientation

        self.move_group.set_pose_target(pose_target)
        self.move_group.set_planning_time(5.0)  # Set planning time in seconds
        self.move_group.set_num_planning_attempts(5)  # Set number of planning attempts
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        rospy.sleep(1)  # Simulate gripper closing time

        # Set the robot to the same pose but with a modified y coordinate
        pose_target.pose.position.x = 0.85
        pose_target.pose.position.y = 0  # Modify the y coordinate
        pose_target.pose.position.z = 0.2
        self.move_group.set_pose_target(pose_target)
        self.move_group.set_planning_time(5.0)  # Set planning time in seconds
        self.move_group.set_num_planning_attempts(5)  # Set number of planning attempts
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()



    def execute_cb(self, goal):
        rospy.loginfo("Received goal: %s", goal)

        rows = goal.rows
        cols = goal.cols

        for i in range(rows):
            for j in range(cols):
                marker_id = i * cols + j
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "my_namespace"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = goal.first_x + j * 0.05
                print(marker.pose.position.x)
                marker.pose.position.y = goal.first_y - i * 0.035
                print(marker.pose.position.y)
                marker.pose.position.z = goal.first_z + 0.2
                print(marker.pose.position.z)
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

