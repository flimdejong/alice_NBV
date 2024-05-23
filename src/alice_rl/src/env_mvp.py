#! /usr/bin/python3.8

import gymnasium
from gymnasium import spaces
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
import sys
import moveit_commander
import numpy as np
import matplotlib.pyplot as plt

#Imports the octomap.srv file from alice_octomap so we can call the server
from alice_octomap.srv import octomap  


class RobotArmEnv(gymnasium.Env):

    def __init__(self):

        # Initialize ROS node
        rospy.init_node('env', anonymous=True)

        # Create a client to call to octoStateServer
        self.octomap_client = rospy.ServiceProxy('octoStateServer', octomap)

        # Dimensions of octomap (in voxels). Currently it is 40 by 40 since
        # we have resolution of 0.05 and 2 by 2 meters bounding box
        self.x = 44
        self.y = 44
        self.total_voxels = 32000

        #Initialize steps taken
        self.steps_taken = 0

        # Define the observation space: discrete position of arm, amount voxels mapped, 2D rep of the octomap.
        # We use a Dict space to combine the discrete and box spaces.

        self.observation_space = spaces.Dict({
            'arm_position': spaces.Discrete(22),
            'voxels_mapped': spaces.Box(low=0, high=self.total_voxels, shape=(1,), dtype=int),
            'map': spaces.Box(low=0, high=1, shape=(self.x, self.y), dtype=int)
        })
        
        
        # Define action space. For now we 21 options along 3 hemispheres.
        # 0 = l1, 1 = l2, 2 = l3, 3 = l4, 4 = l5, 5 = l6
        # 6 = m1

        self.action_space = spaces.Discrete(21)

        # Initialize the MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)

        
    def calculate_reward(self):

        # Penalize amount of steps taken
        step_penalty = self.steps_taken
        voxels_mapped = self.observation_space["voxels_mapped"]

        #Penalize distance between arm states

        # w is weight
        w1 = 1
        w2 = -0.1

        # Calculate the reward linearly based on the number of seen voxels
        reward = w1*voxels_mapped + w2*step_penalty

        return reward

    def print_map(self, map_data):
            plt.figure(figsize=(8, 8))
            plt.imshow(map_data, cmap='binary', origin='lower')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Octomap Representation')
            plt.show(block=False)
            plt.pause(3)
            plt.show()

    def get_eef_distance(self):

        #WIP

        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Get pose of EEF
        current_pose = move_group.get_current_pose()

        # Get position
        eef_position = current_pose.pose.position

        x = eef_position.x
        y = eef_position.y
        z = eef_position.z

        #WIP


    def get_observation_space(self):
        
        # Create a 2D array to represent the x, y map
        map_data = np.zeros((self.x, self.y), dtype=np.int8)

        # If the movement is successful, get the new state
        try:
            
            #res contains the response information from the server octoStateServer
            res = self.octomap_client()

            #Return the amount of occupied voxels
            occupied_voxels = res.occupied_voxels
            rospy.loginfo("Amount of occupied voxels: %d", occupied_voxels)

            #Build a 2D representation of the octomap
            # x = res.x_values
            # y = res.y_values

            # for i in range(len(x)):
            #     map_data[x[i], y[i]] = 1
        
            # Print the map
            #self.print_map(map_data)
        
        except:
            rospy.logerr("Service call failed")

        #Update observation space
        self.observation_space = {
        'arm_position': self.arm_position,
        'voxels_mapped': occupied_voxels,
        'map': map_data
        }

        return self.observation_space
        

    def move_joints(self, target_pose):

        robot = moveit_commander.RobotCommander()
        group_name = "arm"  # Replace with your robot's move group name
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Set the reference frame and end effector link
        move_group.set_pose_reference_frame("base_cyllinder")  # Replace with your robot's base frame
        move_group.set_end_effector_link("camera_link")  # Replace with your end effector link

        # Set the target pose for the arm
        move_group.set_named_target(target_pose)

        # Plan and execute the motion
        plan = move_group.go(wait=True)

        # Check if the planning and execution were successful
        if plan:
            rospy.loginfo("Motion planning and execution succeeded!")
        else:
            rospy.logerr("Motion planning and execution failed!")

        #Clearing poses
        move_group.clear_pose_targets()

        
    def step(self, action):

        action_map = {
            0: "l1",
            1: "l2",
            2: "l3",
            3: "l4",
            4: "l5",
            5: "l6",
            6: "l7",

            7: "m1",
            8: "m2",
            9: "m3",
            10: "m4",
            11: "m5",
            12: "m6",
            13: "m7",
            
            14: "h1",
            15: "h2",
            16: "h3",
            17: "h4",
            18: "h5",
            19: "h6",
            20: "h7",
            21: "Home"
        }

        #For now we use a dictionary to map the numbers to a string
        action_string = action_map.get(action)

        #Keep track of the state we are moving towards
        self.arm_position = action

        #Move the joints to the desired position
        self.move_joints(action_string)

        # When movement is successful, get the new observation space
        self.get_observation_space()

        # Specify the agent has taken another step
        self.steps_taken += 1

        #Calculate the reward
        reward = self.calculate_reward()

        #Placeholder for info
        info = {}

        terminated = self.is_terminated()
        truncated = self.is_truncated()

        return self.observation_space, reward, terminated, truncated, info
        
    
    def reset(self, seed=None):
        # Reset the environment to an initial state

        rospy.loginfo("Resetting state")

        self.move_joints("Home")

        rospy.loginfo("Resetting octomap")


        #Here we reset the state of our script
        self.observation_space['arm_position'] = np.array([21], dtype=int)
        self.observation_space['voxels_mapped'] = np.array([0], dtype=int)
        self.observation_space['map'] = np.zeros((len(self.x), len(self.y)), dtype=int)


        return self.observation_space, {}
    
    def render(self, mode="human"):
        # Implement the rendering logic if required
        pass
    
    def close(self):
        
        # Shutdown the MoveIt commander
        self.move_group.stop()
        moveit_commander.roscpp_shutdown()