#!/usr/bin/env python

import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from geometry_msgs.msg import Pose
import json

def load_json(filename):
    with open(filename, 'r') as file:
         data = json.load(file)
    return data

class RobotMotionController:
    def __init__(self):
        
        # rospy.Subscriber('visualization_marker_array', MarkerArray, queue_size=0)  # You might want to subscribe to this topic if needed

        # Initialize MoveIt Commander
        self.robot = RobotCommander()
        self.move_group = MoveGroupCommander("gripper")

    def go_to_pose_goal(self, pose_goal):
        move_group = self.move_group
        print(move_group.get_current_pose().pose)
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        

def main():
	rospy.init_node('robot_motion_controller', anonymous=True)
	rate = rospy.Rate(1)

	json_data = load_json('/home/yoann/catkin_ws/src/test/src/bonding.json')

	picking_data = json_data["picking"]
	placing_data = json_data["placing"]

	controller = RobotMotionController()  # Creating an instance of the RobotMotionController

	while not rospy.is_shutdown():
		rows = picking_data["magnet_per_row"]
		cols = picking_data["number_of_row"]

		for i in range(rows):
			for j in range(cols):
				move_group = MoveGroupCommander("gripper")
				pose_goal = move_group.get_current_pose().pose
				pose_goal.position.x = picking_data["first_magnet"]["x"] + j * 0.05
				pose_goal.position.y = picking_data["first_magnet"]["y"] - i +0.5
				pose_goal.position.z = picking_data["first_magnet"]["z"] + 0.05
				

				controller.go_to_pose_goal(pose_goal)  # Calling the method through the instance

		        

		rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

