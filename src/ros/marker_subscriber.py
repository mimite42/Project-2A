import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

if __name__ == "__main__":
    rospy.init_node("marker_susbcriber")
    vis_topic = "/visualize_markers"

    markersPub = rospy.Publisher(vis_topic, MarkerArray, queue_size=1)
    markerArray = MarkerArray()

    # Your list of points
    points = [[0.405, -0.107, -0.005],[35.405, -0.107, -0.005],[70.405, -0.107, -0.005],[105.405, -0.107, -0.005],[140.405, -0.107, -0.005],[175.405, -0.107, -0.005],[0.405, 49.893, -0.005],[35.405, 49.893, -0.005],[70.405, 49.893, -0.005],[105.405, 49.893, -0.005],[140.405, 49.893, -0.005],[175.405, 49.893, -0.005],[0.405, 99.893, -0.005],[35.405, 99.893, -0.005],[70.405, 99.893, -0.005],[105.405, 99.893, -0.005],[140.405, 99.893, -0.005],[175.405, 99.893, -0.005],[0.405, 149.893, -0.005],[35.405, 149.893, -0.005],[70.405, 149.893, -0.005],[105.405, 149.893, -0.005],[140.405, 149.893, -0.005],[175.405, 149.893, -0.005],[0.405, 199.893, -0.005],[35.405, 199.893, -0.005],[70.405, 199.893, -0.005],[105.405, 199.893, -0.005],[140.405, 199.893, -0.005],[175.405, 199.893, -0.005]]

    point_list = []

    for i, point in enumerate(points):
        x, y, z = point
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        point_list.append(p)

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        markerArray.markers.clear()
        cmpt = 0
        for p in point_list:
            marker = Marker()
            marker.id = cmpt
            marker.header.frame_id = "base_link"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
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

