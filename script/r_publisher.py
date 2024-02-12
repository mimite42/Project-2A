#!/usr/bin/env python

import sys
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface  # Import moveit_commander
import json
import moveit_commander

# Global set to store processed marker IDs
processed_markers = set()

def load_json(filename):
    with open(filename, 'r') as file:
        data = json.load(file)
    return data

def move_to_position(move_group, x, y, z):
    initial_joint_values = move_group.get_current_joint_values()

    pose_target = PoseStamped()
    pose_target.header.frame_id = "base_link"
    pose_target.pose.position.x = x
    pose_target.pose.position.y = y
    pose_target.pose.position.z = 0.2
    pose_target.pose.orientation.w = 1

    # Assuming no rotation
    current_orientation = move_group.get_current_pose().pose.orientation
    pose_target.pose.orientation = current_orientation

    move_group.set_pose_target(pose_target)
    move_group.set_planning_time(5.0)  # Set planning time in seconds
    move_group.set_num_planning_attempts(5)  # Set number of planning attempts
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

  
    rospy.sleep(4.5)  # Simulate gripper closing time

    # Set the robot to the same pose but with a modified y coordinate
    pose_target.pose.position.y = -y  # Modify the y coordinate
    move_group.set_pose_target(pose_target)
    move_group.set_planning_time(5.0)  # Set planning time in seconds
    move_group.set_num_planning_attempts(5)  # Set number of planning attempts
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

 

def main():
    rospy.init_node('r_publisher')
    vis_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=0)
    rate = rospy.Rate(1)  # 1 Hz

    moveit_commander.roscpp_initialize(sys.argv)
    robot = RobotCommander()
    move_group = MoveGroupCommander("arm")
   
    scene = PlanningSceneInterface()

    json_data = load_json('/home/yoann/catkin_ws/src/test/src/bonding.json')

    picking_data = json_data["picking"]
    placing_data = json_data["placing"]

    while not rospy.is_shutdown():
        marker_array = MarkerArray()

        rows = picking_data["magnet_per_row"]
        cols = picking_data["number_of_row"]

        for i in range(rows):
            for j in range(cols):
                marker_id = i * cols + j
                if marker_id in processed_markers:
                    continue  # Skip if marker has already been processed

                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = rospy.Time()
                marker.ns = "my_namespace"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = picking_data["first_magnet"]["x"] + j * 0.05
                marker.pose.position.y = picking_data["first_magnet"]["y"] - i * 0.035
                marker.pose.position.z = picking_data["first_magnet"]["z"] + 0.5
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                # Move to position and close/open gripper
                move_to_position(move_group, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)

                # Simulate grasping time
                rospy.sleep(2.0)

                # Simulate placing time and add marker
                marker.pose.position.z = 0.1
                marker_array.markers.append(marker)

                # Change the marker ID before adding it to processed_markers
                new_marker_id = marker_id + 1000  # You can use any strategy to create a new ID
                processed_markers.add(new_marker_id)

                # Delete the marker after processing
                delete_marker(vis_pub, [marker_id])

        # Add all the processed markers to the MarkerArray
        add_marker(vis_pub, marker_array.markers)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


