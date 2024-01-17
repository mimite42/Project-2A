#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import test.msg
import json

def main():
    rospy.init_node("test_Move")

    client = actionlib.SimpleActionClient(
        "Move", ACROBA_Workshop_SIGMA.msg.MoveAction
    )
    client.wait_for_server()
    goal = ACROBA_Workshop_SIGMA.msg.MoveGoal()
    rate = rospy.Rate(60)
    json_data = load_json('/home/younes/drari/Project-2A/src/bonding.json')

    magnet_length = json_data["Magnet"]["length"]
    magnet_width = json_data["Magnet"]["width"]
    magnet_thickness = json_data["Magnet"]["thickness"]

    picking_data = json_data["picking"]
    placing_data = json_data["placing"]
    while not rospy.is_shutdown():
    
        goal.rows = picking_data["magnet_per_row"]
        goal.cols = picking_data["number_of_row"]
        goal.first_mag_x = picking_data["first_magnet"]["x"]
        goal.first_mag_y = picking_data["first_magnet"]["y"]
        goal.first_mag_z = picking_data["first_magnet"]["z"]

        client.send_goal(goal)
        client.wait_for_result()
        rate.sleep()


if __name__ == "__main__":
    main()
