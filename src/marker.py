#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def main():
    rospy.init_node('marker_publisher')
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=0)
    rate = rospy.Rate(1)  # 1 Hz

    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time()
    marker.ns = "my_namespace"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x =0.405
    marker.pose.position.y = -0.107
    marker.pose.position.z = -0.005
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.01  # Diamètre de la sphère en mètres (10 mm)
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.a = 1.0
    marker.color.r = 1.0  # Rouge
    marker.color.g = 0.0
    marker.color.b = 0.0

    while not rospy.is_shutdown():
        vis_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

