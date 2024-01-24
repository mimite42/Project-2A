#!/usr/bin/env python3
import rospy
import actionlib
import test.msg
import json

def load_json(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)

class MoveActionServer:
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            test.msg.MoveAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("Server Ready...")

    def execute_cb(self, goal):
        json_data = load_json(goal.path)
        picking_data = json_data["picking"]
        result = test.msg.MoveResult(
            rows=picking_data["magnet_per_row"],
            cols=picking_data["number_of_row"],
            first_mag_x=picking_data["first_magnet"]["x"],
            first_mag_y=picking_data["first_magnet"]["y"],
            first_mag_z=picking_data["first_magnet"]["z"]
        )

        # Here you can implement the logic to execute the action

        self._as.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("get_data_json")
    server = MoveActionServer(rospy.get_name())
    rospy.spin()

