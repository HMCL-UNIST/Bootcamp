###########################################################
##########     TEST CODE FOR RRT ASSIGNMENT      ##########
###########################################################

# This is the file for testing your RRT code
# RRT Assignment will be graded based on this code
# You can edit this file for debugging
# but final score will be graded by running the original code,
# so please be sure to test with the original file

import time
import gym
import numpy as np
import concurrent.futures
import os
import sys
import matplotlib.pyplot as plt

# Get ./src/ folder & add it to path
current_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(current_dir)

# import your drivers here
from drivers_ppc_rrt import PurePursuitDriver

# choose your drivers here (1-4)
drivers = [PurePursuitDriver()]

# choose your racetrack here
RACETRACK = 'SIMPLE'

#import your planner here
from planner_rrt import RRTPlanner

class GymRunner(object):

    def __init__(self, racetrack, drivers):
        self.racetrack = racetrack
        self.drivers = drivers
        self.map_points = np.load("./map_rrt.npy")
        self.start = (0, 0)  # Starting node
        self.goal = (-15, 20)  # Goal node
        self.planner = RRTPlanner(self.start, self.goal, self.map_points)

    def run(self):
        # load map
        env = gym.make('f110_gym:f110-v0',
                       map="{}/maps/{}".format(current_dir, RACETRACK),
                       map_ext=".png", num_agents=len(drivers))

        # specify starting positions of each agent
        poses = np.array([[0. + (i * 0.75), 0. - (i*1.5), np.radians(60)] for i in range(len(drivers))])

        obs, step_reward, done, info = env.reset(poses=poses)
        env.render()

        #################################################
        ############# DO NOT TOUCH THIS CODE#############
        #################################################
        laptime = 0.0
        checker = 0.0
        check_start = False
        check_goal = False
        check_path = True
        final_point = 0
        start = time.time()

        plt.figure(figsize=(10,20))
        plt.cla()

        ref = self.planner.rrt_planning()

        if ref[0,0] == self.start[0] and ref[0,1] == self.start[1]:
            check_start = True
        if ref[-1,0] == self.goal[0] and ref[-1,1] == self.goal[1]:
            check_goal = True
        for i in range(len(ref)-1):
            if (pow((ref[i+1][0] - ref[i][0]), 2) + pow((ref[i+1][1] - ref[i][1]), 2)) > 3:
                check_path = False
                break
        if(check_start and check_goal and check_path):
            print("Path is generated successfully!\nYou got 2 points!")
            final_point = 2
        #################################################
        #################################################

        while not done:
            actions = []
            futures = []
            with concurrent.futures.ThreadPoolExecutor() as executor:
                for i, driver in enumerate(drivers):
                    futures.append(executor.submit(driver.pure_pursuit_control, obs["poses_x"][0], obs["poses_y"][0], obs["poses_theta"][0], ref))
            for future in futures:
                speed, steer, l_idx = future.result()
                actions.append([steer, speed])
            actions = np.array(actions)
            obs, step_reward, done, info = env.step(actions)
            laptime += step_reward
            env.render(mode='human')

            ###########################################################
            ##########      Plot Tools for Debugging        ###########
            ##########  Uncomment lines below to use plot   ###########
            ###########################################################
            if laptime - checker > 0.1:
                checker = laptime
                plt.clf()
                plt.plot(self.map_points[0,:],self.map_points[1,:],"*k", markersize = 0.5)
                plt.plot(ref[:,0],ref[:,1],"*b", markersize = 1)
                plt.plot(ref[l_idx,0],ref[l_idx,1],"*g", markersize = 5)
                plt.plot(obs["poses_x"][0],obs["poses_y"][0],"*r", markersize = 5)
                plt.plot(self.start[0],self.start[1],"*b", markersize = 10)
                plt.plot(self.goal[0],self.goal[1],"*y", markersize = 10)
                plt.show(block=False)
                plt.pause(0.1)

            #################################################
            ############# DO NOT TOUCH THIS CODE#############
            #################################################
            dist_to_goal = pow((self.goal[0] - obs["poses_x"][0]), 2) + pow((self.goal[1] - obs["poses_y"][0]), 2)
            if dist_to_goal < 0.5:
                final_point = 5
                print("Your car has successfully reached the goal!\nYou got 5 points! Congratulations!!")
                break
            if laptime > 30:
                print("Your simulation time exceeds 30 seconds. Simulation Done.")
            #################################################
            #################################################

        print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time() - start, 'Final Score :', final_point)

if __name__ == '__main__':
    runner = GymRunner(RACETRACK, drivers)
    runner.run()
