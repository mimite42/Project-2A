#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray

def create_marker(x, y, z, marker_id):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time()
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
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
    return marker

def pos_mark_callback(msg):
    global vis_pub

    # Assuming the data format is [x1, y1, z1, x2, y2, z2, ...]
    coordinates = msg.data
    num_markers = len(coordinates) // 3

    marker_array = MarkerArray()

    for i in range(num_markers):
        x = coordinates[i * 3]
        y = coordinates[i * 3 + 1]
        z = coordinates[i * 3 + 2]
        marker = create_marker(x, y, z, i)
        marker_array.markers.append(marker)
    print(marker_array)
    vis_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        rospy.init_node('marker_subscriber')
         # Subscribe to the "pos_mark" topic
        rospy.Subscriber('pos_mark', Float32MultiArray, pos_mark_callback)
        
        vis_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=0)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

