#!/usr/bin/env python3

import rospy
import actionlib
import test.msg

class Patrol:

    feedback_ = test.msg.ur_robotFeedback()
    result_ = test.msg.ur_robotResult()

    client_json = actionlib.SimpleActionClient(
        "get_data_json", test.msg.MoveAction
    )

    client_Robot = actionlib.SimpleActionClient(
        "Robot", test.msg.RobotAction
    )

    goal_json = test.msg.MoveGoal()
    goal_Robot = test.msg.RobotGoal()

    def __init__(self, name):
        self.client_json.wait_for_server()
        self.client_Robot.wait_for_server()

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            test.msg.ur_robotAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("Server sk ready")

    def execute_cb(self, goal):
        rospy.loginfo("Executing %s" % self._action_name)
        rate = rospy.Rate(60)
        success = True

        self.goal_json.path = goal.sk_path
        self.client_json.send_goal(self.goal_json)

        rospy.loginfo("Waiting for Move_2 action server to complete...")
        self.client_json.wait_for_result()
        rospy.loginfo("Move_2 action server completed.")

        result_json = self.client_json.get_result()

        if success:
            self.goal_Robot.rows = result_json.rows
            self.goal_Robot.cols = result_json.cols
            self.goal_Robot.first_x = result_json.first_mag_x
            self.goal_Robot.first_y = result_json.first_mag_y
            self.goal_Robot.first_z = result_json.first_mag_z
            self.client_Robot.send_goal(self.goal_Robot)

            rospy.loginfo("Waiting for Robot action server to complete...")
            self.client_Robot.wait_for_result()
            rospy.loginfo("Robot action server completed.")

            if self.client_Robot.get_state() == actionlib.GoalStatus.ABORTED:
                success = False
                rospy.loginfo("%s: Aborted" % self._action_name)
                self._as.set_aborted()

        if success:
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self.result_)

if __name__ == "__main__":
    rospy.init_node("robot_skill")
    server = Patrol(rospy.get_name())
    rospy.spin()

