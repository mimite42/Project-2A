#!/usr/bin/env python

import rospy
import actionlib
from test.msg import PoseAction, PoseGoal, PoseResult
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def handle_pose_action(goal):
    rospy.loginfo("Received goal: {}".format(goal.pose))

    # Your logic to process the goal and generate a result
    # For illustration purposes, let's just concatenate the coordinates
    marker_state = " ".join(map(str, goal.pose))

    # Create and fill the result message
    result = PoseResult()
    result.marker_state = marker_state

    # Publish the result
    server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("pose_action_server")

    # Create the action server
    server = actionlib.SimpleActionServer('pose_action', PoseAction, handle_pose_action, auto_start=False)
    server.start()

    vis_topic = "/visualize_markers"
    markers_pub = rospy.Publisher(vis_topic, MarkerArray, queue_size=1)
    marker_array = MarkerArray()

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        marker_array.markers.clear()

        # Define your PoseGoal using point_list
        point_list = PoseGoal()
        cmpt = 0
        for cmpt in range(len(point_list.pose)):
            p = point_list.pose[cmpt]

            marker = Marker()
            marker.id = cmpt
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = point_list.pose[p*3]
            marker.pose.position.y = point_list.pose[p*3+1]
            marker.pose.position.z = point_list.pose[p*3+2]
            marker_array.markers.append(marker)
            cmpt += 1 

        rospy.loginfo("publishing marker")
        markers_pub.publish(marker_array)
        rate.sleep()

    rospy.spin()

