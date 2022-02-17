from operator import truediv
from pickletools import uint8
from re import X
import time
from typing import List, Tuple, Union
from copy import copy
import numpy as np
import pygame
from math import sin, cos, atan, atan2, pi, sqrt
import cv2

def rot_points(mat, radians: float):
    rot = []
    rot_mat = np.array([[cos(radians), sin(radians)],[-sin(radians), cos(radians)]])

    for m in mat:
        rot.append(m @ rot_mat)
    return rot

class Robot:
    x_coord: int
    y_coord: int
    width: int = 70
    height: int = 70
    leg_length: int = 30
    wheel_diam: int = 10
    angle = 0

    # leg angles in degrees wrt robot frame
    # wheel angles in degrees wrt robot frame

    leg: list() = [45., 90., 180., 60.]
    wheel: list() = [45., 45., 45., 45.]

    


    def __init__(self, x, y) -> None:
        self.x_coord = x
        self.y_coord = y


class World:    

    obstacle = []

    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height

        


class Visualizer:
    BLACK: Tuple[int, int, int] = (0, 0, 0)
    RED: Tuple[int, int, int] = (255, 0, 0)
    WHITE: Tuple[int, int, int] = (255, 255, 255)
    BLUE: Tuple[int, int, int] = (0, 0, 255)

    def __init__(self, robot: Robot, world: World) -> None:
        pygame.init()
        pygame.font.init()
        self.robot = robot
        self.world = world
        self.screen = pygame.display.set_mode((world.width, world.height))
        pygame.display.set_caption('Tetromino Challenge')
        self.font = pygame.font.SysFont('freesansbolf.tff', 30)
    
    def display_robot(self):
        xor = [[1,-1], [-1,-1], [-1,1], [1,1]]
        pygame.draw.circle(self.screen, self.RED, (self.robot.x_coord, self.robot.y_coord),2)
        pos_x = self.robot.x_coord - self.robot.width/2
        pos_y = self.robot.y_coord - self.robot.height/2
        
        pygame.draw.rect(self.screen, self.RED, (pos_x, pos_y, self.robot.width, self.robot.height), 2)
        
        # leg
        for i in range(4):
            
            start_x = self.robot.x_coord + xor[i][0]*self.robot.width/2
            start_y = self.robot.y_coord + xor[i][1]*self.robot.height/2
            angle_leg = 2*pi - self.robot.leg[i] * pi/180
            
            line_leg = rot_points([[0,0],[self.robot.leg_length,0]], angle_leg) + np.array([[start_x, start_y],[start_x, start_y]])
            pygame.draw.line(self.screen, self.BLACK, line_leg[0], line_leg[1], 2)

            angle_wheel = 2*pi - self.robot.wheel[i] * pi/180
            line_wheel = rot_points([[-self.robot.wheel_diam,0],[self.robot.wheel_diam,0]], angle_wheel) + np.array([line_leg[1],line_leg[1]])
            pygame.draw.line(self.screen, self.BLUE, line_wheel[0], line_wheel[1], 2)

    def display_world(self):
        """ use this for obstacles """
        pass


    def update_display(self) -> bool:

        self.screen.fill(self.WHITE)

        self.display_world()

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

        while running:

            running = self.vis.update_display()

            time.sleep(0.1)
        

def main():
    height = 1000
    width = 1000

    robot = Robot(300,400)
    world = World(width, height)
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
