import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

if __name__ == "__main__":
    rospy.init_node("test_pub")
    vis_topic = "/visualize_markers"

    markersPub = rospy.Publisher(vis_topic, MarkerArray, queue_size=1)
    markerArray = MarkerArray()

    point1, point2, point3 = Point(), Point(), Point()
    point_list = []

    point1.x = 0
    point1.y = 0
    point1.z = 0

    point2.x = 1
    point2.y = 1
    point2.z = 1

    point3.x = 2
    point3.y = 2
    point3.z = 2

    point_list.append(point1)
    point_list.append(point2)
    point_list.append(point3)

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        markerArray.markers.clear()
        cmpt = 0
        for p in point_list:
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
            marker.pose.position.x = p.x
            marker.pose.position.y = p.y
            marker.pose.position.z = p.z
            markerArray.markers.append(marker)
            cmpt += 1
        rospy.loginfo("publishing marker")
        markersPub.publish(markerArray)
        rate.sleep()

