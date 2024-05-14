#! /usr/bin/env python
import gymnasium
from gymnasium import spaces
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
import sys
import moveit_commander

#Imports the octomap.srv file from alice_octomap so we can call the server
from alice_octomap.srv import octomap  


class RobotArmEnv(gymnasium.Env):

    def __init__(self):

        # Initialize ROS node
        rospy.init_node('env', anonymous=True)

        # Create a client to call to octoStateServer
        self.octomap_client = rospy.ServiceProxy('octoStateServer', octomap)
        
        # Initialize the state variable
        self.state = self.get_state()
        
        # Define the observation space (quantity of seen voxels)
        self.observation_space = spaces.Box(low=0, high=1000, shape=(1,), dtype=int)
        
        # Define the action space (joint angles)
        self.action_space = spaces.Box(low=0.5, high=1.5, shape=(4,), dtype=float)

        # Initialize the MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)

        
    def calculate_reward(self,state):
        seen_voxels = state

        if seen_voxels >= 100:
            # Reward for seeing a high number of voxels
            reward = 10.0
            
        elif seen_voxels >= 50:
            # Reward for seeing a moderate number of voxels
            reward = 5.0
        else:
            # Penalty for seeing a low number of voxels
            reward = -5

    def get_state(self):
        # If the movement is successful, get the new state
        try:
            response = self.octomap_client()
            occupied_voxels = response.occupied_voxels
            rospy.loginfo("Amount of occupied voxels: %d", occupied_voxels)
            #rospy.loginfo("This is a test to check if this line runs")

            total_voxels = response.total_voxels
            rospy.loginfo("Amount of total voxels: %d", total_voxels)
        
        except:
            rospy.logerr("Service call failed")
        
        self.state = occupied_voxels
        rospy.loginfo("state is: ")
        rospy.loginfo(self.state)

    def move_joints(self, target_joint_angles):

        robot = moveit_commander.RobotCommander()
        group_name = "arm"  # Replace with your robot's move group name
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Set the reference frame and end effector link
        move_group.set_pose_reference_frame("base_cyllinder")  # Replace with your robot's base frame
        move_group.set_end_effector_link("camera_link")  # Replace with your end effector link

        # Set the start state to the current state
        # start_state = robot.get_current_state()
        # move_group.set_start_state(start_state)

        # Set the joint angles for the arm
        move_group.set_joint_value_target(target_joint_angles)

        # Plan and execute the motion
        plan = move_group.go(wait=True)

        # Check if the planning and execution were successful
        if plan:
            rospy.loginfo("Motion planning and execution succeeded!")
        else:
            rospy.logerr("Motion planning and execution failed!")

        
        
    def step(self, action):

        #Apply the action
        #Actions are the joint positions (base_joint, shoulder_joint, elbow_joint, wrist_pitch_joint)



        # #We create a ROS publisher to send the information to our moveIt service node.
        # joint_pub = rospy.Publisher('joint_pub_rl', Float32MultiArray, queue_size=10)

        # #Populate the joint information with our action
        # joint_pub_msg = Float32MultiArray
        # joint_pub_msg.data = action

        # #Publish the information
        # joint_pub.publish(joint_pub_msg)

        # # Log the published message
        # rospy.loginfo("Published joint data: %s", joint_pub_msg.data)

        # # Shutdown the ROS node to only publish once per step
        # rospy.signal_shutdown("Joint data published. Shutting down the node.")

        self.move_joints(action)

        # If the movement is successful, get the new state
        self.get_state()
        
        #Calculate the reward
        reward = self.calculate_reward(self.state)

        #Placeholder for info
        info = {}

        return self.state, reward, info
        
    
    def reset(self, seed=None):
        # Reset the environment to an initial state

        rospy.loginfo("Resetting")

        initial_pos = [0.0, 1.5708, 0.0, 0.0]
        self.move_joints(initial_pos)

        while self.state != 80:
            rospy.loginfo("Waiting for initial state...")
            rospy.sleep(1.0)

        return self.state, {}
    
    def render(self, mode='human'):
        # Implement the rendering logic if required
        pass
    
    def close(self):
        # Close any open resources or connections

        
        # Shutdown the MoveIt commander
        self.move_group.stop()
        moveit_commander.roscpp_shutdown()