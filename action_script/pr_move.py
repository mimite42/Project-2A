#!/usr/bin/env python3
import rospy
import actionlib
from test.msg import MoveAction, MoveFeedback, MoveResult  # Updated import

class MoveActionServer:
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            MoveAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("Server Ready...")

    def execute_cb(self, goal):
        rospy.loginfo("rows: %s", goal.rows)
        rospy.loginfo("cols: %s", goal.cols)
        rospy.loginfo("first_mag_x: %s", goal.first_mag_x)
        rospy.loginfo("first_mag_y: %s", goal.first_mag_y)
        rospy.loginfo("first_mag_z: %s", goal.first_mag_z)

        # Here you can implement the logic to execute the action

        result = MoveResult()
        self._as.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("pr_move")
    server = MoveActionServer(rospy.get_name())
    rospy.spin()

