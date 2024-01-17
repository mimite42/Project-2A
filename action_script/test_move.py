#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import ACROBA_Workshop_SIGMA.msg


<<<<<<< HEAD
=======


>>>>>>> ddf142b06b599f103f84a0ac3891f2628dd630ea
def main():
    rospy.init_node("test_Move")

    client = actionlib.SimpleActionClient(
        "Move", ACROBA_Workshop_SIGMA.msg.MoveAction
    )
    client.wait_for_server()
    goal = ACROBA_Workshop_SIGMA.msg.MoveGoal()
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
<<<<<<< HEAD
        goal.turtle_name = "turtle1"
        goal.speed = 0.1
=======
        goal.turtlename = "turtle1"
        goal.vitesse = 2.5
>>>>>>> ddf142b06b599f103f84a0ac3891f2628dd630ea
        goal.distance = 2
        goal.isForward = True
        client.send_goal(goal)
        client.wait_for_result()
        rate.sleep()


if __name__ == "__main__":
    main()
