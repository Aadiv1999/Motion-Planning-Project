from itertools import count
from operator import truediv
from pickletools import uint8
from re import X
import time
from turtle import heading
from typing import List, Tuple, Union
from copy import copy
import cv2
import numpy as np
import pygame
import random
from math import sin, cos, atan, atan2, pi, sqrt
from astar import AStarPlanner

HEIGHT = 900
WIDTH = 900

def rot_points(mat, degrees: float):
    rot = []
    degrees = degrees * pi/180
    rot_mat = np.array([[cos(degrees), sin(degrees)],[-sin(degrees), cos(degrees)]])

    return mat @ rot_mat

def convert_to_display(mat):
    mat[:,1] = HEIGHT - mat[:,1]
    return mat

class Robot:
    x_coord: int
    y_coord: int
    width: int = 20
    height: int = 20
    leg_length: int = 5
    wheel_diam: int = 4
    angle = 0
    points = []
    # leg angles in degrees wrt robot frame
    # wheel angles in degrees wrt robot frame
    # this will be rotated
    leg = np.array([45., 135., 225., 315.])
    wheel = np.array([315., 45., 135., 225.])   

    visited = []


    def __init__(self, x, y) -> None:
        self.x_coord = x
        self.y_coord = y
        # self.chassis = self.robot_points()
    
    def get_robot_points(self):
        points = []
        
        # chassis points
        # top right index 0
        points.append([int(self.width/2), int(self.height/2)])
        # top left index 1
        points.append([-int(self.width/2), int(self.height/2)])
        # bottom left index 2
        points.append([-int(self.width/2), -int(self.height/2)])
        # bottom right index 3
        points.append([int(self.width/2), -int(self.height/2)])

        # legs
        # index 4 to 7
        for i in range(4):
            leg_points = rot_points([[0,0], [self.leg_length,0]], self.leg[i])[1] + np.array(points[i])
            points.append(leg_points)

        # wheels
        for i in range(4):
            wheel_points = rot_points([[-self.wheel_diam,0], [self.wheel_diam,0]], self.wheel[i]) + np.array([points[i+4], points[i+4]])
            points.append(wheel_points[0])
            points.append(wheel_points[1])

        heading = [[0,0],[0,self.height]]
        points.append(heading[0])
        points.append(heading[1])
        points = rot_points(points, self.angle) + np.array([self.x_coord, self.y_coord])

        points = convert_to_display(points)
        return points
    
    def turn_to(self, theta: float) -> None:
        self.wheel[:] = theta
    
    def set_position(self, x: int, y:int) -> None:
        self.x_coord = x
        self.y_coord = y


class World:    
    #obstacle = []
    tree_xs = []
    tree_ys = []
    tree_sizes = []

    cropRow_widths = []
    crop_xs = []
    cropWidth_total = 0
    cropRow_spacing: int
    prev_crop_width = 0
    prev_crop_x = 0

    cropsPerRow: int
    vert_crop_spacing: float
    crop_space_ys = []

    weed_xs = []
    weed_ys = []
    weed_sizes = []

    world_array = []

    def __init__(self, width: int, height: int, numTrees: int, numCropRows: int, cropsPerRow: int, numWeeds: int) -> None:
        self.width = width
        self.height = height
        self.numTrees = numTrees
        self.numCropRows = numCropRows
        self.cropsPerRow = cropsPerRow
        self.numWeeds = numWeeds
        for i in range(numTrees):
            x_pos = random.randint(0, width)
            y_pos = random.randint(0, height)
            tree_radius = random.randint(10, 25)
            self.tree_xs.append(x_pos)
            self.tree_ys.append(y_pos)
            self.tree_sizes.append(tree_radius)
        for i in range(numCropRows):
            row_width = random.randint(5,20)
            self.cropRow_widths.append(row_width)
            self.cropWidth_total = self.cropWidth_total + row_width
        self.cropRow_spacing = int((width - self.cropWidth_total)/(numCropRows + 2))
        for i in range(numCropRows):
            x_pos = self.cropRow_spacing + self.prev_crop_width + self.prev_crop_x
            self.prev_crop_width = self.cropRow_widths[i-1]
            self.prev_crop_x = x_pos
            self.crop_xs.append(x_pos)
        for i in range(numWeeds):
            x_pos = random.randint(0, width)
            y_pos = random.randint(0, height)
            weed_radius = random.randint(5, 15)
            self.weed_xs.append(x_pos)
            self.weed_ys.append(y_pos)
            self.weed_sizes.append(weed_radius)
        self.vert_crop_spacing = self.height/((self.cropsPerRow*2)-1)
        print(self.vert_crop_spacing)
        for i in range((cropsPerRow*2)+1):
            # print(i)
            if i == 0:
                y_pos = 0
            elif (i%2) == 0:
                y_pos = (i-1) * self.vert_crop_spacing
                self.crop_space_ys.append(y_pos)
            else:
                y_pos = (i-1) * self.vert_crop_spacing


class Planner:

    trajectory = []
    trajectoryTotal = []
    def __init__(self, robot: Robot, world: World) -> None:
        self.robot = robot
        self.world = world
    
    def get_path(self, start: Tuple[int, int], goal: Tuple[int, int]):
        obs = self.world.world_array
        # obs[1:10, 1:10] = 255
        
        obs_idx = np.argwhere(obs == 255)
        ox = np.array(obs_idx[:,0])
        oy = np.array(obs_idx[:,1])
        obs_map = obs

        

        
        astar = AStarPlanner(ox, oy, obs_map)
        rx, ry = astar.planning(start[0], start[1], goal[0], goal[1])
        obs[rx, ry] = 255
        x = HEIGHT//300
        # print("RX: ",rx)
        self.trajectory = np.flip(np.vstack((ry*x, rx*x)).T, axis=0)
        # self.trajectory = np.flip(np.vstack((rx*x, ry*x)).T, axis=0)

        # cv2.imshow('map', obs_map)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
    


class Visualizer:
    BLACK: Tuple[int, int, int] = (0, 0, 0)
    RED: Tuple[int, int, int] = (255, 0, 0)
    WHITE: Tuple[int, int, int] = (255, 255, 255)
    BLUE: Tuple[int, int, int] = (0, 0, 255)
    TREE_GREEN: Tuple[int, int, int] = (34, 139, 34)
    CROP_GREEN: Tuple[int, int, int] = (167, 199, 155)
    WEED_GREEN: Tuple[int, int, int] = (20, 66, 6)

    def __init__(self, robot: Robot, world: World, planner: Planner) -> None:
        pygame.init()
        pygame.font.init()
        self.robot = robot
        self.world = world
        self.planner = planner
        self.screen = pygame.display.set_mode((world.width, world.height))
        pygame.display.set_caption('Farmland')
        self.font = pygame.font.SysFont('freesansbolf.tff', 30)
    
    def display_robot(self):
        angle = 0

        all_points = self.robot.get_robot_points()
        
        # plot chassis
        for i in range(4):
            if i == 3:
                pygame.draw.line(self.screen, self.RED, all_points[i], all_points[0], 4)
            else:
                pygame.draw.line(self.screen, self.RED, all_points[i], all_points[i+1], 4)

        # plot legs
        for i in range(4):
            pygame.draw.line(self.screen, self.BLUE, all_points[i], all_points[i+4], 2)
        
        # plot wheels
        for i in range(8,15,2):
            pygame.draw.line(self.screen, self.BLACK, all_points[i], all_points[i+1], 4)
        
        # heading
        pygame.draw.line(self.screen, self.BLUE, all_points[-2], all_points[-1], 2)

    def display_world(self):
        for i in range(self.world.numCropRows):
            x_pos = self.world.crop_xs[i]
            crop_width = self.world.cropRow_widths[i]
            y_pos = 0
            pygame.draw.rect(self.screen, self.CROP_GREEN, (x_pos, y_pos, crop_width, self.world.height))
        for i in range(len(self.world.crop_space_ys)):
            y_pos = self.world.crop_space_ys[i]
            width = self.world.vert_crop_spacing
            x_pos = 0
            pygame.draw.rect(self.screen, self.WHITE, (x_pos, y_pos, self.world.width, width))
        for i in range(self.world.numWeeds):
            x_pos = self.world.weed_xs[i]
            y_pos = self.world.weed_ys[i]
            weed_radius = self.world.weed_sizes[i]
            pygame.draw.circle(self.screen, self.WEED_GREEN, (x_pos, y_pos), weed_radius)
        for i in range(self.world.numTrees):
            x_pos = self.world.tree_xs[i]
            y_pos = self.world.tree_ys[i]
            tree_radius = self.world.tree_sizes[i]
            pygame.draw.circle(self.screen, self.TREE_GREEN, (x_pos, y_pos), tree_radius)

    def display_environment(self):
        for i in range(self.world.numCropRows):
            x_pos = self.world.crop_xs[i]
            crop_width = self.world.cropRow_widths[i]
            y_pos = 0
            pygame.draw.rect(self.screen, self.CROP_GREEN, (x_pos, y_pos, crop_width, self.world.height))
        
        for i in range(len(self.world.crop_space_ys)):
            y_pos = self.world.crop_space_ys[i]
            width = self.world.vert_crop_spacing
            x_pos = 0
            pygame.draw.rect(self.screen, self.WHITE, (x_pos, y_pos, self.world.width, width))

        for i in range(self.world.numTrees):
            x_pos = self.world.tree_xs[i]
            y_pos = self.world.tree_ys[i]
            tree_radius = self.world.tree_sizes[i]
            pygame.draw.circle(self.screen, self.TREE_GREEN, (x_pos, y_pos), tree_radius)
        
    def display_trajectory(self):
        
        for i in range(len(self.planner.trajectoryTotal)-1):
            pygame.draw.line(self.screen, self.RED, self.planner.trajectoryTotal[i], self.planner.trajectoryTotal[i+1], 1)

    def update_display(self, counter: int) -> bool:

        self.screen.fill(self.WHITE)

        self.display_world()

        self.display_trajectory()

        if counter < 1:
            self.screen.fill(self.WHITE)
            self.display_environment()
            pygame.image.save(self.screen, 'output.png')
            img = cv2.threshold(255-cv2.imread('output.png',cv2.IMREAD_GRAYSCALE), 50, 255, cv2.THRESH_BINARY)[1]
            img_resize = cv2.resize(img, (300, 300), interpolation=cv2.INTER_AREA)
            kernel = np.ones((5,5), np.uint8)
            self.world.world_array = cv2.dilate(img_resize, kernel, iterations=1)
            cv2.imwrite('output_dilated.png', self.world.world_array)
            # cv2.imshow('world', self.world.world_array)
            # cv2.waitKey()
            # cv2.destroyAllWindows()

        if counter > 5:
            self.display_robot()

        for i in range(len(self.robot.visited)-1):
            pygame.draw.line(self.screen, self.BLUE, self.robot.visited[i], self.robot.visited[i+1], 1)

        for event in pygame.event.get():
            # Keypress
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: # if escape is pressed, quit the program
                    return False

        pygame.display.flip()

        return True

    def cleanup(self) -> None:
        pygame.quit()


class Runner:
    def __init__(self, robot: Robot, world: World, vis: Visualizer, planner: Planner) -> None:
        self.robot = robot
        self.world = world
        self.vis = vis
        self.planner = planner
        
        

    def run(self):
        running = True
        counter = 0
        weeds_destroyed = 0

        while running:
            running = self.vis.update_display(counter)
            
            if counter == 0:
                # self.planner.get_path((290, 290), (0, 0))
                # trajectoryTotal = self.planner.trajectory
                # self.planner.trajectoryTotal = trajectoryTotal
                for j in range(0,len(self.world.weed_xs)):
                    if j == 0:
                        self.planner.get_path((290, 290), (self.world.weed_ys[j]/3, self.world.weed_xs[j]/3))
                        trajectoryTotal = self.planner.trajectory
                        self.planner.trajectoryTotal = trajectoryTotal
                    else:
                        self.planner.get_path((self.world.weed_ys[j-1]/3, self.world.weed_xs[j-1]/3), (self.world.weed_ys[j]/3, self.world.weed_xs[j]/3))
                        trajectoryTotal = np.vstack([trajectoryTotal, self.planner.trajectory])
                        self.planner.trajectoryTotal = trajectoryTotal
                # print(trajectoryTotal)
                # print(len(self.planner.trajectory))
            

            
            if counter < len(self.planner.trajectoryTotal)-1:
                x = self.planner.trajectoryTotal[counter][0]
                x_next = self.planner.trajectoryTotal[counter+1][0]
                y = self.planner.trajectoryTotal[counter][1]
                y_next = self.planner.trajectoryTotal[counter+1][1]

                angle = atan2(y_next-y, x_next-x)
                
                if abs(angle) % pi/2 == 0:
                    self.robot.turn_to(angle*180/pi)
                else:
                    self.robot.turn_to(angle*180/pi + 90)
                self.robot.set_position(x,HEIGHT-y)
                self.robot.visited.append([x,y])
                # print(len(self.robot.visited))

            counter += 1
            time.sleep(0.05)
        

def main():
    height = HEIGHT
    width = WIDTH
    numTrees = 3
    #numCropRows = 7
    #cropsPerRow = 10
    numCropRows = 5
    cropsPerRow = 10
    numWeeds = 3

    robot = Robot(500,500)
    world = World(width, height, numTrees, numCropRows, cropsPerRow, numWeeds)
    planner = Planner(robot, world)
    vis = Visualizer(robot, world, planner)

    runner = Runner(robot, world, vis, planner)

    try:
        runner.run()
    except AssertionError as e:
        print(f'ERROR: {e}, Aborting.')
    except KeyboardInterrupt:
        pass
    finally:
        vis.cleanup()


if __name__ == '__main__':
    main()
