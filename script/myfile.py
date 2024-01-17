#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String

def move_robot():
    # Initialize the moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('myfile', anonymous=True)

    # Create a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # Create a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()

    # Create a MoveGroupCommander object for the end effector
    move_group = moveit_commander.MoveGroupCommander("end_eff")

    # Set the planning time
    move_group.set_planning_time(5)

    # Set a joint state goal (adjust the joint values as needed)
    joint_goal = [0, -1.57, 1.57, -1.57, -1.57, 0]
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # Plan and execute a Cartesian path
    waypoints = []
    scale = 1
    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += scale * 0.2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    move_group.execute(plan, wait=True)

    # Clean up MoveIt Commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass

