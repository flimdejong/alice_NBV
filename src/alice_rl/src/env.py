#! /usr/bin/env python

import gymnasium
from gymnasium import spaces
import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from alice_octomap.srv import octomap_srv_client


class RobotArmEnv(gymnasium.Env):
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('env', anonymous=True)

        # Create a service client for the octomap_srv_client service (yes client to client)
        self.octomap_client = rospy.ServiceProxy('octomap_binary', octomap_srv_client)
        
        # Initialize the state variable
        self.state = self.get_state()
        
        # Define the observation space (quantity of seen voxels)
        self.observation_space = spaces.Box(low=0, high=1000, shape=(1,), dtype=int)
        
        # Define the action space (joint angles)
        self.action_space = spaces.Box(low=0, high=3.14, shape=(4,), dtype=float)

        

    def calculate_reward(self,state):
        seen_voxels = state

        if seen_voxels >= 200:
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
        
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        self.state = occupied_voxels

        
    def step(self, action):

        #Apply the action
        #Actions are the joint positions (base_joint, shoulder_joint, elbow_joint, wrist_pitch_joint)

        #We create a ROS publisher to send the information to our moveIt service node.
        joint_pub = rospy.Publisher('joint_pub_rl', Float32MultiArray, queue_size=10)

        #Populate the joint information with our action
        joint_pub_msg = Float32MultiArray
        joint_pub_msg.data = action

        #Publish the information
        joint_pub.publish(joint_pub_msg)

        # Log the published message
        rospy.loginfo("Published joint data: %s", joint_pub_msg.data)

        # Shutdown the ROS node to only publish once per step
        rospy.signal_shutdown("Joint data published. Shutting down the node.")

        # If the movement is successful, get the new state
        self.get_state()
        
        #Calculate the reward
        reward = self.calculate_reward(self.state)

        #Placeholder for info
        info = {}

        return self.state, reward, info
        
    
    def reset(self):
        # Reset the environment to an initial state
        # You can publish a reset command to the appropriate topic if needed
        # For example:
        # reset_msg = ...  # Create a reset message
        # self.reset_pub.publish(reset_msg)
        
        # Wait for a certain duration or until the initial state is received
        # You can use rospy.sleep() or rospy.wait_for_message() depending on your requirements
        
        # Return the initial observation
        return self.state
    
    def render(self, mode='human'):
        # Implement the rendering logic if required
        pass
    
    def close(self):
        # Close any open resources or connections
        pass