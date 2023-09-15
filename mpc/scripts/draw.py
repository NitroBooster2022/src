#!/usr/bin/env python
# coding=utf-8

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches


class Draw_MPC_point_stabilization_v1(object):
    def __init__(self, robot_states: list, init_state: np.array, target_state: np.array, rob_diam=0.3,
                 export_fig=False):
        self.robot_states = robot_states
        self.init_state = init_state
        self.target_state = target_state
        self.rob_radius = rob_diam / 2.0
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(-0.8, 3), ylim=(-0.8, 3.))
        # self.fig.set_dpi(400)
        self.fig.set_size_inches(7, 6.5)
        # init for plot
        self.animation_init()

        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, range(len(self.robot_states)),
                                           init_func=self.animation_init, interval=100, repeat=False)

        plt.grid('--')
        if export_fig:
            self.ani.save('./v1.gif', writer='imagemagick', fps=100)
        plt.show()

    def animation_init(self):
        # plot target state
        self.target_circle = plt.Circle(self.target_state[:2], self.rob_radius, color='b', fill=False)
        self.ax.add_artist(self.target_circle)
        self.target_arr = mpatches.Arrow(self.target_state[0], self.target_state[1],
                                         self.rob_radius * np.cos(self.target_state[2]),
                                         self.rob_radius * np.sin(self.target_state[2]), width=0.2)
        self.ax.add_patch(self.target_arr)
        self.robot_body = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.robot_body)
        self.robot_arr = mpatches.Arrow(self.init_state[0], self.init_state[1],
                                        self.rob_radius * np.cos(self.init_state[2]),
                                        self.rob_radius * np.sin(self.init_state[2]), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        return self.target_circle, self.target_arr, self.robot_body, self.robot_arr

    def animation_loop(self, indx):
        position = self.robot_states[indx][:2]
        orientation = self.robot_states[indx][2]
        self.robot_body.center = position
        # self.ax.add_artist(self.robot_body)
        self.robot_arr.remove()
        self.robot_arr = mpatches.Arrow(position[0], position[1], self.rob_radius * np.cos(orientation),
                                        self.rob_radius * np.sin(orientation), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        return self.robot_arr, self.robot_body


class Draw_MPC_Obstacle(object):
    def __init__(self, robot_states: list, init_state: np.array, target_state: np.array, obstacle: np.array,
                 grid: np.array, rob_diam=3.0, export_fig=None, xmin=-3.2, xmax=3.2, ymin=-3.2, ymax=3.2):
        self.robot_states = robot_states
        self.init_state = init_state
        self.target_state = target_state
        self.rob_radius = rob_diam / 2.0
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(xmin, xmax), ylim=(ymin, ymax))
        if obstacle is not None:
            self.obstacle = obstacle
        else:
            print('no obstacle given, break')
        self.fig.set_size_inches(7, 6.5)
        self.obstacle_circles = [] 
        
        # Create colored patches for non-zero grid cells
        self.grid_patches = []
        print("grid shape: ", grid.shape)
        print(grid)
        # flip grid to match plot
        grid = grid.T
        # grid = np.flip(grid, 1)
        width = xmax/grid.shape[0]
        height = ymax/grid.shape[1]
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                if grid[i, j] != 0:
                    patch = plt.Rectangle((i*width, j*height), width, height, color='green') 
                    self.grid_patches.append(patch)
                    
        # init for plot
        self.animation_init()

        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, range(len(self.robot_states)),
                                           init_func=self.animation_init, interval=100, repeat=False)

        plt.grid('--')
        if export_fig is not None:
            self.ani.save(export_fig+'.gif', writer='imagemagick', fps=100)
        plt.show()

    def animation_init(self):
        # Add grid patches to the plot
        for patch in self.grid_patches:
            self.ax.add_patch(patch)
            
        # plot target state
        self.target_circle = plt.Circle(self.target_state[:2], self.rob_radius, color='b', fill=False)
        self.ax.add_artist(self.target_circle)
        self.target_arr = mpatches.Arrow(self.target_state[0], self.target_state[1],
                                         self.rob_radius * np.cos(self.target_state[2]),
                                         self.rob_radius * np.sin(self.target_state[2]), width=0.2)
        self.ax.add_patch(self.target_arr)
        self.robot_body = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.robot_body)
        self.robot_arr = mpatches.Arrow(self.init_state[0], self.init_state[1],
                                        self.rob_radius * np.cos(self.init_state[2]),
                                        self.rob_radius * np.sin(self.init_state[2]), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        
        for obs in self.obstacle:  # Iterate over each obstacle
            obs_circle = plt.Circle(obs[:2], obs[2], color='purple', fill=True)
            self.ax.add_artist(obs_circle)
            self.obstacle_circles.append(obs_circle)
        
        return self.target_circle, self.target_arr, self.robot_body, self.robot_arr, self.obstacle_circles

    def animation_loop(self, indx):
        position = self.robot_states[indx][:2]
        orientation = self.robot_states[indx][2]
        self.robot_body.center = position
        self.robot_arr.remove()
        self.robot_arr = mpatches.Arrow(position[0], position[1], self.rob_radius * np.cos(orientation),
                                        self.rob_radius * np.sin(orientation), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        return self.robot_arr, self.robot_body


class Draw_MPC_tracking(object):
    def __init__(self, robot_states: list, init_state: np.array, grid: np.array, obstacle: np.array, rob_diam=0.3,  export_fig=None
                 , xmin=-1.0, xmax=15, ymin=-1, ymax=15, waypoints_x = None, waypoints_y = None, spline_points_x = None, spline_points_y = None):
        self.init_state = init_state
        self.robot_states = robot_states
        self.rob_radius = rob_diam
        self.waypoints_x = waypoints_x
        self.waypoints_y = waypoints_y
        self.spline_points_x = spline_points_x
        self.spline_points_y = spline_points_y
        import cv2
        self.image = cv2.imread('/home/simonli/Documents/husky_ws/src/control/scripts/imgs/Competition_track_graph.png')
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.image = np.array(self.image).T
        # resize the image to 500 by 500
        self.image = cv2.resize(self.image, (100, 100), interpolation=cv2.INTER_AREA)
        print(self.image.shape)

        if obstacle is not None:
            self.obstacle = obstacle
        else:
            print('no obstacle given, break')
        self.fig = plt.figure()
        # self.ax = plt.axes(xlim=(-1.0, 15), ylim=(-1, 15))
        self.ax = plt.axes(xlim=(xmin, xmax), ylim=(ymin, ymax))
        # self.fig.set_size_inches(7, 6.5)
        self.obstacle_circles = [] 

        self.background_image = plt.imread('/home/simonli/Documents/husky_ws/src/control/scripts/imgs/Competition_track_graph.png') 

        self.grid_patches = []
        if grid is not None:
            grid = self.image
            print("grid shape: ", grid.shape)
            print(grid)
            # flip grid to match plot
            grid = grid.T
            # grid = np.flip(grid, 1)
            width = xmax/grid.shape[0]
            height = ymax/grid.shape[1]
            for i in range(grid.shape[0]):
                for j in range(grid.shape[1]):
                    if grid[i, j] != 0:
                        patch = plt.Rectangle((i*width, j*height), width, height, color='green') 
                        self.grid_patches.append(patch)

        # init for plot
        self.animation_init()

        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, range(len(self.robot_states)),
                                           init_func=self.animation_init, interval=100, repeat=False)

        plt.grid('--')
        if export_fig is not None:
            self.ani.save(export_fig+'.gif', writer='imagemagick', fps=100)
        plt.show()

    def animation_init(self, ):
        if self.spline_points_x is not None:
            self.ax.plot(self.spline_points_x, self.spline_points_y, '-g', label='Continuous Path')
        for patch in self.grid_patches:
            self.ax.add_patch(patch)
        if self.waypoints_x is not None:
            self.ax.plot(self.waypoints_x[:], self.waypoints_y[:], 'go', label='Waypoints',markersize=1)
            # draw waypoints smaller circle
        
        # self.ax.imshow(self.background_image, extent=[0, 15, 0, 15], aspect='auto')

        # draw target line
        # self.target_line = plt.plot([0, 12], [1, 1], '-r')
        # t_ref_line = np.linspace(-1, 357, 357*5)  # 400 points between -1 and 15
        # x_ref_line = 0.5 * t_ref_line
        # y_ref_line = np.sin(t_ref_line)+3
        # self.target_line, = self.ax.plot(x_ref_line, y_ref_line, '-r')  # Plot the reference line

        # draw the initial position of the robot
        self.init_robot_position = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.init_robot_position)
        self.robot_body = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.robot_body)
        self.robot_arr = mpatches.Arrow(self.init_state[0], self.init_state[1],
                                        self.rob_radius * np.cos(self.init_state[2]),
                                        self.rob_radius * np.sin(self.init_state[2]), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        for obs in self.obstacle:  # Iterate over each obstacle
            obs_circle = plt.Circle(obs[:2], obs[2], color='purple', fill=True)
            self.ax.add_artist(obs_circle)
            self.obstacle_circles.append(obs_circle)
        return
        # return self.target_line, self.init_robot_position, self.robot_body, self.robot_arr, self.obstacle_circles

    def animation_loop(self, indx):
        position = self.robot_states[indx][:2]
        orientation = self.robot_states[indx][2]
        self.robot_body.center = position
        self.robot_arr.remove()
        self.robot_arr = mpatches.Arrow(position[0], position[1], self.rob_radius * np.cos(orientation),
                                        self.rob_radius * np.sin(orientation), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        return self.robot_arr, self.robot_body