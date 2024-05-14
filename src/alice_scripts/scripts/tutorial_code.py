#! /usr/bin/env python

#Include the necessary libraries 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from math import pi
from sensor_msgs.msg import JointState

class MyRobot:

    # Default Constructor
    def __init__(self, Group_Name):

        #Initialize the moveit_commander and rospy node
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_redefined_pose', anonymous=True)

        
        #Instantiate a RobotCommander object. This object is the outer-level interface to the robot
        self._robot = moveit_commander.RobotCommander()
        #Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
        self._scene = moveit_commander.PlanningSceneInterface()
        
        #define the movegoup for the robotic 
        #Replace this value with your robots planning group name that you had set in Movit Setup Assistant
        self._planning_group = Group_Name
        #Instantiate a MoveGroupCommander Object. This Object is an interface to the one group of joints. this interface can be used to plan and execute the motions on the robotic arm
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        
        #We create a DisplayTrajectory publisher which is used later to publish trajectories for RViz to visualize
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        #Create action client for the Execute Trajectory action server
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        #Get the planning frame, end effector link and the robot group names
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        #print the info
        #here the '\033[95m' represents the standard colour "LightMagenta" in terminals. For details, refer: https://pkg.go.dev/github.com/whitedevops/colors
        #The '\033[0m' is added at the end of string to reset the terminal colours to default
        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        
        #for moveit_commander member functions in Python 3 (For Noetic), please refer: https://docs.ros.org/en/noetic/api/moveit_commander/html/functions_func.html
        #for moveit_commander member functions in Python 2, please refer(For Kinetic or Melodic): https://docs.ros.org/en/kinetic/api/moveit_commander/html/functions_func.html
        #Python file with function definitions: https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
        #Python file with function definitions (for Noetic): https://docs.ros.org/en/noetic/api/moveit_commander/html/move__group_8py_source.html
        #Python file with function definitions (for Kinetic or Melodic): https://docs.ros.org/en/kinetic/api/moveit_commander/html/move__group_8py_source.html

        #Set the predefined position as the named joint configuration as the goal to plan for the move group. The predefined positions are defined in the Moveit Setup Assistant
        self._group.set_named_target(arg_pose_name)
        #Plan to the desired joint-space goal using the default planner
        plan = self._group.plan()
        #Create a goal message object for the action server
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        #Update the trajectory in the goal message
        goal.trajectory = plan
        #Send the goal to the action server
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        #Print the current pose
        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    

    # Class Destructor
    def __del__(self):
        #When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')


    def set_joint_value(self, joint_name, joint_value):

        # Set the start state to the current state
        self._group.set_start_state_to_current_state()
        
        rospy.loginfo('\033[32m' + "Moving joint: {} to position: {}".format(joint_name, joint_value) + '\033[0m')
        
        # Set the joint value as the target
        self._group.set_joint_value_target(joint_name, joint_value)
        
        # Plan the motion
        plan = self._group.plan()
        
        # Create a RobotTrajectory object from the plan
        robot_trajectory = plan[1]
        
        # Create a goal message object for the action server
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        
        # Set the trajectory in the goal message
        goal.trajectory = robot_trajectory
        
        # Send the goal to the action server
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        
        # Execute the planned motion synchronously
        success = self._group.execute(robot_trajectory, wait=True)
        
        if success:
            rospy.loginfo('\033[32m' + "Joint: {} moved to position: {}".format(joint_name, joint_value) + '\033[0m')
        else:
            rospy.loginfo('\033[31m' + "Failed to execute the motion for joint: {}".format(joint_name) + '\033[0m')


def main():

    #Create a new arm object from the MyRobot class
    arm = MyRobot("arm")
    
    # Move a specific joint to a desired position

    arm.set_joint_value("wrist_pitch_joint", 1.57)
    rospy.sleep(15)

    arm.set_joint_value("base_joint", 1.57)


    rospy.sleep(40)
    #delete the arm object at the end of code
    del arm
	

if __name__ == '__main__':
    main()