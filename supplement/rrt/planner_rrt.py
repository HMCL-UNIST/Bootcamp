
# ASSIGNMENT - Implement the planner using RRT algorithm 

import numpy as np
import math

class Node:

    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

class RRTPlanner:

    def __init__(self, s_start, s_goal, map_points):

        #################################################
        ############# DO NOT TOUCH THIS CODE#############
        #################################################
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.x_range = (-23.8, 2.2)
        self.y_range = (-3.0, 19.0)
        self.map = map_points
        #################################################
        #################################################    

        # you should set these value appropriately   
        self.step_len = 
        self.goal_sample_rate = 
        self.iter_max = 

        # you can add some more class variables under this line
        # self.variable = value
        # like this

    def rrt_planning(self):

        # TODO implement RRT algorithm

        path = np.zeros(shape=(100,2))

        # find path from start pose to goal pose which does not collide with map
        # sample node from the in the range self.x_range, self.y_range

        return path
    
    # you can define more functions if you want