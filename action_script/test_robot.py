#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import test.msg

def main():
    rospy.init_node("test_skill_node")

    client = actionlib.SimpleActionClient(
        "robot_skill", test.msg.ur_robotAction
    )
    client.wait_for_server()
    goal = test.msg.ur_robotGoal()
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        goal.sk_path = '/home/younes/drari/Project-2A/src/bonding.json'

        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()  # Access the result received from the server
        rospy.loginfo("rows: %s", result.rows)
        rospy.loginfo("cols: %s", result.cols)
        rospy.loginfo("first_mag_x: %s", result.first_mag_x)
        rospy.loginfo("first_mag_y: %s", result.first_mag_y)
        rospy.loginfo("first_mag_z: %s", result.first_mag_z)
        rate.sleep()

if __name__ == "__main__":
    main()

