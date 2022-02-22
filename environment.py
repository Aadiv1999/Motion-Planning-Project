from operator import truediv
from pickletools import uint8
from re import X
import time
from turtle import heading
from typing import List, Tuple, Union
from copy import copy
import numpy as np
import pygame
import random
from math import sin, cos, atan, atan2, pi, sqrt
#import cv2

def rot_points(mat, degrees: float):
    rot = []
    degrees = degrees * pi/180
    rot_mat = np.array([[cos(degrees), sin(degrees)],[-sin(degrees), cos(degrees)]])

    return mat @ rot_mat

def convert_to_display(mat):
    mat[:,1] = 1000 - mat[:,1]
    return mat

class Robot:
    x_coord: int
    y_coord: int
    width: int = 100
    height: int = 100
    leg_length: int = 50
    wheel_diam: int = 20
    angle = 0
    points = []
    # leg angles in degrees wrt robot frame
    # wheel angles in degrees wrt robot frame
    # this will be rotated
    leg = np.array([45., 135., 225., 315.])
    wheel = np.array([315., 45., 135., 225.])   


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
            tree_radius = random.randint(20, 70)
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

        


class Visualizer:
    BLACK: Tuple[int, int, int] = (0, 0, 0)
    RED: Tuple[int, int, int] = (255, 0, 0)
    WHITE: Tuple[int, int, int] = (255, 255, 255)
    BLUE: Tuple[int, int, int] = (0, 0, 255)
    TREE_GREEN: Tuple[int, int, int] = (34, 139, 34)
    CROP_GREEN: Tuple[int, int, int] = (167, 199, 155)
    WEED_GREEN: Tuple[int, int, int] = (20, 66, 6)

    def __init__(self, robot: Robot, world: World) -> None:
        pygame.init()
        pygame.font.init()
        self.robot = robot
        self.world = world
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


    def update_display(self) -> bool:

        self.screen.fill(self.WHITE)

        # self.display_world()

        self.display_robot()

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
    def __init__(self, robot: Robot, world: World, vis: Visualizer) -> None:
        self.robot = robot
        self.world = world
        self.vis = vis
        
        

    def run(self):
        running = True
        counter = 0

        while running:
            

            self.robot.angle += 1
            self.robot.y_coord += 1

            running = self.vis.update_display()

            counter += 1
            time.sleep(0.01)
        

def main():
    height = 1000
    width = 1000
    numTrees = 10
    numCropRows = 20
    cropsPerRow = 30
    numWeeds = 50

    robot = Robot(500,500)
    world = World(width, height, numTrees, numCropRows, cropsPerRow, numWeeds)
    vis = Visualizer(robot, world)

    runner = Runner(robot, world, vis)

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
