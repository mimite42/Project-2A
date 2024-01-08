#!/usr/bin/env python

import sys
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface  # Import moveit_commander
import json
import moveit_commander

def load_json(filename):
    with open(filename, 'r') as file:
        data = json.load(file)
    return data

def move_to_position(move_group, x, y, z):
	pose_target = PoseStamped()
	pose_target.header.frame_id = "base_link"
	pose_target.pose.position.x = x
	pose_target.pose.position.y = y
	pose_target.pose.position.z = 0.1
	pose_target.pose.orientation.w = 1
	# Assuming no rotation
	current_orientation = move_group.get_current_pose().pose.orientation
	pose_target.pose.orientation = current_orientation
	move_group.set_pose_target(pose_target)
	plan = move_group.go(wait=True)
	move_group.stop()
	move_group.clear_pose_targets()

def main():
    rospy.init_node('marker_publisher')
    vis_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=0)
    rate = rospy.Rate(1)  # 1 Hz

    moveit_commander.roscpp_initialize(sys.argv)
    robot = RobotCommander()
    move_group = MoveGroupCommander("ur_robot")
    scene = PlanningSceneInterface()

    json_data = load_json('/home/yoann/catkin_ws/src/test/src/bonding.json')

    magnet_length = json_data["Magnet"]["length"]
    magnet_width = json_data["Magnet"]["width"]
    magnet_thickness = json_data["Magnet"]["thickness"]

    picking_data = json_data["picking"]
    placing_data = json_data["placing"]

    while not rospy.is_shutdown():
        marker_array = MarkerArray()

        rows = picking_data["magnet_per_row"]
        cols = picking_data["number_of_row"]

        for i in range(rows):
            for j in range(cols):
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = rospy.Time()
                marker.ns = "my_namespace"
                marker.id = i * cols + j  # Unique ID for each marker
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = (
                    picking_data["first_magnet"]["x"] + j * 0.05
                )
                marker.pose.position.y = (
                    picking_data["first_magnet"]["y"] - i * 0.035
                )
                marker.pose.position.z = picking_data["first_magnet"]["z"] + 0.5
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.01  # Convert to meters
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r = 1.0  # Red
                marker.color.g = 0.0
                marker.color.b = 0.0

                # Move the robot to the marker position
                move_to_position(move_group, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)

                marker_array.markers.append(marker)

        vis_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

