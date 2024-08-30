#
#
# DO NOT MODIFY THIS FILE.
#
# Please, do not modify this file. This file implements the behaviour of the obstacles, and therefore it
# cannot be changed, as that could be used to make the task much easier.
#
#
import os
import math
import time
import random
from controller import Robot


robot = Robot()

timestep = int(robot.getBasicTimeStep())

lms291 = robot.getDevice("Sick LMS 291")
lms291.enable(timestep)
lms291_width = lms291.getHorizontalResolution()

front_left_wheel = robot.getDevice("front left wheel")
front_right_wheel = robot.getDevice("front right wheel")
back_left_wheel = robot.getDevice("back left wheel")
back_right_wheel = robot.getDevice("back right wheel")
front_left_wheel.setPosition(math.inf)
back_left_wheel.setPosition(math.inf)
front_right_wheel.setPosition(math.inf)
back_right_wheel.setPosition(math.inf)
front_left_wheel.setVelocity(0)
back_left_wheel.setVelocity(0)
front_right_wheel.setVelocity(0)
back_right_wheel.setVelocity(0)


OBSTACLE_THRESHOLD = 0.4


rotating = False
rotating_timestamp = None
rotating_time = None

while (robot.step(timestep) != -1):

    # Not rotating
    if not rotating:
        lms291_values = lms291.getRangeImage()
        # Check transition into rotating
        for d in lms291_values[len(lms291_values)//3 : -len(lms291_values)//3]:
            if d < OBSTACLE_THRESHOLD:
                rotating = True
                rotating_timestamp = time.time()
                rotating_time = random.randint(3,8)
        if not rotating:
            front_left_speed = 2
            front_right_speed = 2
            back_left_speed = 2
            back_right_speed = 2
    # Rotating    
    if rotating:
        if time.time() < rotating_timestamp + rotating_time:
            back_left_speed = 1
            front_left_speed = 1
            back_right_speed = -1
            front_right_speed = -1
        else:
            rotating = False

    # set actuators
    front_left_wheel.setVelocity(front_left_speed)
    back_left_wheel.setVelocity(back_left_speed)
    front_right_wheel.setVelocity(front_right_speed)
    back_right_wheel.setVelocity(back_right_speed)
    









