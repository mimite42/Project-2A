#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray

def main():
    rospy.init_node('marker_publisher')
    vis_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=0)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        marker_array = MarkerArray()

        rows = 6
        cols = 5

        for i in range(rows):
            for j in range(cols):
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = rospy.Time()
                marker.ns = "my_namespace"
                marker.id = i * cols + j  # Unique ID for each marker
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = 0.405 + j * 0.05  # Adjust the x position for each marker
                marker.pose.position.y = -0.107 - i * 0.035  # Adjust the y position for each marker
                marker.pose.position.z = 0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.01  # Diameter of the sphere in meters (10 mm)
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r = 1.0  # Red
                marker.color.g = 0.0
                marker.color.b = 0.0

                marker_array.markers.append(marker)

        vis_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

