#!/usr/bin/env python3
import sys
import rospy
from math import radians
from math import degrees
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import numpy as np
import random
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState

class Project3:

    def __init__(self):
        self.init_node()
        self.init_subscriber()
        self.start_gazebo()
        if "train" in sys.argv[1:]:
            self.train()
        if "test" in sys.argv[1:]:
            self.test_prompt()

    def init_node(self):
        ### Initialize the ROS node here
        rospy.init_node("task2", anonymous= True)


    def init_subscriber(self):
        ### Initialize the subscriber 
        self.subscribe = rospy.Subscriber("/scan", LaserScan, self.callback)

    def start_gazebo(self):
        ###Gets all needed services from gazebo

        self.reset = rospy.ServiceProxy("/gazebo/reset simulation", Empty)
        self.get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self. set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause_physics = rospy.ServiceProxy("gazebo/pause_physics", Empty)
    
    def store_scanner_data(self, data):
        ###Sets the range data to class object variable ranges
        self.ranges = data.ranges
        self.range_min = data.range_min
        self.inc = data.angle_increment 

    def find_state(self):
               ### Uses range data to determine states

        ranges = self.ranges
        min_range = self.range_min
        increment = self.inc
        new_state = []
        minimum_distanceL = 1000
        minimum_distanceR = 1000
        minimum_distanceRF = 1000
        minimum_distanceF = 1000

        #replace invalid min with infinity
        for i in range(len(ranges)):
            if ranges[i] < min_range:
                ranges[i] = float('inf')
        
        for i in range(len(ranges)):
            angle = degrees(i* increment)
            
            #checking Left
            if angle >= 75 and angle <= 105: #Cone of 30
                if  ranges[i] < minimum_distanceL:
                    minimum_distanceL = ranges[i]
            
            #checking Right
            if angle >= 255 and angle <= 285: #Cone of 30
                if  ranges[i] < minimum_distanceR:
                    minimum_distanceR = ranges[i]
            #Checking RightFront
            if angle >= 300 and angle <= 330: #Cone of 30
                if  ranges[i] < minimum_distanceRF:
                    minimum_distanceRF = ranges[i]

            #checking Front
            if angle >= 0 and angle <= 15: #Cone of 30
                if  ranges[i] < minimum_distanceF:
                    minimum_distanceF = ranges[i]

            #checking Front
            if angle >= 345 and angle <= 360: #Cone of 30
                if  ranges[i] < minimum_distanceF:
                    minimum_distanceF = ranges[i]


        new_state.append(self.get_minimum_distance_index(minimum_distanceL))
        new_state.append(self.get_minimum_distance_index(minimum_distanceR))
        new_state.append(self.get_minimum_distance_index(minimum_distanceRF))
        new_state.append(self.get_minimum_distance_index(minimum_distanceF))
        return tuple(new_state)

    def get_minimum_distance_index(self, minimum_distance):
        ###Gets Uses minimum distance to find corosponding state index
        
        #define state ranges
        close = 0.25 # Close: x < 0.25
        far = 0.6 # Medium 0.25 <= x <= 0.6  #Far: x >0.6
        state_index = 0

        #Figure out range category
        if minimum_distance < close:
            state_index = 0
        elif minimum_distance >= close and minimum_distance <= far:
            state_index = 1
        else:
            state_index = 2

        return state_index
        


    def set_model_state(self, x, y, z):
        ### gets position of robot and sets robot to location/orientation in gazebo

        #Converts from euler cordinate to quaternion
        quaternion = quaternion_from_euler(0, 0, z)

        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_burger'
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 0
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]

        rospy.wait_for_service('/gazebo/set_model_state')
        
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(state_msg )

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def get_model_state(self):
        ###Gets location of robot from gazebo 

        rospy.wait_for_service('/gazebo/get_model_state')
        getting_state = self.get_state("turtlebot3_burger", "")
        return (getting_state.pose.position, getting_state.pose.orientation)

    
    def create_qtable(self):
        self.distances = 3 #Close, Medium, Far
        self.num_actions = 3 #left, Right, Left

        self.q_table = np.zeros((self.distances, self.distances ,self.distances ,self.distances , self.num_actions))
        #self.actions = ['up', 'right', 'left'] As a reference

    def is_terminal_state(self):
        #if roboot too close to wall, terminate
        return self.terminate
    
    #Starting Location of robot
    def get_starting_location(self):

        locations = []
        locations.append((1, -2, 0)) #right wall Facing Up (Right Wall heading towards corner)
        locations.append((-2, -0.3, radians(270))) #bottom wall Facing Right (Bottom Wall heading towards corner)
        locations.append((0,2,radians(180)))  # Left Wall Facing Down (Left Wall heading towards corner)

        locations.append((2, 0, radians(90))) #Top wall Facing Left (Top Wall heading towards U turn)
        locations.append((2, 1, radians(90))) #Top wall Facing Left (Top Wall heading towards U turn, but closer)

        locations.append((-2, 0.3, 0)) #Bottom wall Facing Up (Bottom I turn)
        locations.append((2, -1, radians(180))) #Bottom wall Facing Up (I turn)
        locations.append((-2, -2, 0)) #Right wall Facing Up (I turn Approaching)


        return random.choice(locations)
    

    #define an epsilon greedy algorithm that will choose which action to take next (i.e., where to move next)
    def get_next_action(self,state_index_tuple, epsilon):
        ###Gets the index of the next action based on greedy or random

        #Each corrosponding index of state
        left, right, right_front, front = state_index_tuple

        #Greedy
        if np.random.random() < epsilon:
            self.greedy = True
            return np.argmax(self.q_table[left, right, right_front, front])
        else: #choose a random action (Not Greedy)
            self.greedy = False
            return np.random.randint(self.num_actions)
        

    def move(self, action_index):
        ###Moves robot based on given action index
        movement = Twist()
        x = 0.1
        z = 0.00001

        if action_index == 1: # Right Turn
            z = 1
        if action_index == 2: #Left Turn
            z = -1
            
        movement.linear.x = x
        movement.angular.z = z
        
        #publishes movement to gazebo
        publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        publisher.publish(movement)

    def give_reward(self, states, action):
        ###calculates and returns reward

        #index representations of state
        left, right, right_front, front = states

        #Index of given range categories
        close = 0
        medium = 1
        far = 2

        #default varible used to determine if action was correct
        correct_action = False

        #If robot is close to wall, punish (-10)
        if left == close or right == close or right_front == close or front == close:
            reward = -10
        
        #If robot is far from wall, punish (-10)
        elif right == far:
            reward = -10
        #No punishment for robot that is medium distance to wall (0)
        else:
            reward = 0
        
        #If robot is set to terminate, reward -100 points
        if self.terminate:
            reward = -100
            print("WALL")

        #Correct State/Actions taken
        if (states == (2,1,1,2) and action == 1) or (states == (2,1,2,2) and action == 0) or (states == (2,1,1,1) and action == 2):
            #Based on right wall following implementation

            # Turn right when at corner of vertical piller (I)
            # stay straigh when medium range at wall with no corners
            # Turn left when at Uturn
            correct_action = True

        #If greedy, then add to total greedy actions
        if self.greedy:
            self.greedy_actions_total+= 1

            #if correct action, then add to corret greedy actions
            if correct_action:
                self.greedy_actions_corrct+=1
        return reward

    def train(self):
        #define training parameters
        epsilon = .1 #the percentage of time when we should take the best action (instead of a random action)
        discount_factor = 0.9 #discount factor for future rewards
        learning_rate = 0.15 #the rate at which learn happens
        episode_num = 600 #number of episodes
        self.greedy_actions_total = 0
        self.greedy_actions_corrct = 0
        self.create_qtable() #creates qtable filled with 0 values
        self.ranges = None
        rate = rospy.Rate(3) #Sleep rate
        epsilon_rate = 0.9/episode_num #determines rate at which epsilon increases up to 0.9
        self.previous_ranges = []
        
        #run through training episodes
        for episode in range(episode_num):
            #sets terminate to False at start of episode
            self.terminate = False
            print("EPISODE NUMBER: ", episode)

            #get the starting location of robot for this episode
            current_pos = self.get_starting_location()
            x_loc, y_loc, z_loc = current_pos

            #Sets starting location in gazebo
            self.set_model_state(x_loc, y_loc, z_loc)

            #makes sure ranges is different than previous step and not Null
            while self.ranges is None or self.previous_ranges is self.ranges:
                pass
            
            #While robot is not terminated, episode continues
            while not self.is_terminal_state():

                #Makes sure ranges is uiniqe from previous step
                while self.previous_ranges is self.ranges:
                    pass

                #sets previous range to current ranges
                self.previous_ranges = self.ranges

                #Get Robot's Location
                rob_pos, rob_ori = self.get_model_state()
                rob_ori = euler_from_quaternion((rob_ori.w, rob_ori.x, rob_ori.y, rob_ori.z))
                
                #Get Robot's State
                state_tuple = self.find_state()

                #choose which action to take
                action_index = self.get_next_action(state_tuple, epsilon)

                #Store old state
                old_state_tuple = state_tuple

                #perform the chosen action
                self.move(action_index)
                rate.sleep()

                #Get state of next move
                rob_pos, rob_ori = self.get_model_state()
                current_pos = (rob_pos.x, rob_pos.y, rob_pos.z)
                rob_ori = euler_from_quaternion((rob_ori.w, rob_ori.x, rob_ori.y, rob_ori.z))
               
                #store new state
                state_tuple = self.find_state()

                #receive the reward for moving to the new state, and calculate the temporal difference
                reward =  self.give_reward(state_tuple, action_index)

                #calculate temporal diference
                old_left, old_right, old_right_front, old_front = old_state_tuple
                left, right, right_front, front = state_tuple
                old_q_value = self.q_table[old_left, old_right, old_right_front, old_front, action_index]
                temporal_difference = reward + (discount_factor * np.max(self.q_table[left, right, right_front, front])) - old_q_value

                #update the Q-value for the previous state and action pair
                new_q_value = old_q_value + (learning_rate * temporal_difference)
                self.q_table[old_left, old_right, old_right_front, old_front, action_index] = new_q_value

            #Update elsilon at the end of the episode
            epsilon += epsilon_rate
        
        print('Training complete!')
        if self.greedy_actions_total != 0:
            print("Score:", self.greedy_actions_corrct/self.greedy_actions_total)
        else:
            print("Score:", 0)
        
        args = sys.argv[1:]
        file_name = args[-1]
        np.save(file_name, self.q_table)


    def test_prompt(self):
        ###Gets file name for training
        args = sys.argv[1:]
        file_name = args[-1]
        self.q_table = np.load(file_name)
        print(self.q_table)
        self.test()

    def test(self):
        self.ranges = None
        self.previous_ranges = []
        rate = rospy.Rate(3) #Sleep rate
        
        #sets terminate to False at start of episode
        self.terminate = False
        
        #get the starting location of robot for this episode
        current_pos = self.get_starting_location()
        x_loc, y_loc, z_loc = current_pos

        #Sets starting location in gazebo
        self.set_model_state(x_loc, y_loc, z_loc)

        #makes sure ranges is different than previous step and not Null
        while self.ranges is None or self.previous_ranges is self.ranges:
            pass
            
        #While robot is not terminated, test continues
        while not self.is_terminal_state():
            #Makes sure ranges is uiniqe from previous step
            while self.previous_ranges is self.ranges:
                pass

            #sets previous range to current ranges
            self.previous_ranges = self.ranges

            #Get Robot's Location
            rob_pos, rob_ori = self.get_model_state()
            rob_ori = euler_from_quaternion((rob_ori.w, rob_ori.x, rob_ori.y, rob_ori.z))
                
            #Get Robot's State
            state_tuple = self.find_state()

            #Each corrosponding index of state
            left, right, right_front, front = state_tuple
            #choose which action to take
            action_index = np.argmax(self.q_table[left, right, right_front, front])

            #perform the chosen action
            self.move(action_index)
            rate.sleep()
        print('Testing complete!')


    def callback(self, data):
        self.store_scanner_data(data)
        
if __name__ == "__main__":
    Project3()