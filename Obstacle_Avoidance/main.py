import math
from turtle import width
import pygame
from Robot import *

MAP_DIMENSIONS = (600, 1200)

# Environment graphics
gfx = Graphics(MAP_DIMENSIONS, "robot.png", "map.png")

# Robot
start = (200, 200)
width = 0.01 * 3779.52
robot = Robot(start, width)

# Sensor range
sensorRange = (250, math.radians(40))
ultraSonic = Ultrasonic(sensorRange, gfx.map)

dt = 0
lastTime = pygame.time.get_ticks()


# Starting
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    dt = (pygame.time.get_ticks() - lastTime) / 1000
    lastTime = pygame.time.get_ticks()

    gfx.map.blit(gfx.mapImg, (0, 0))

    robot.kinematics(dt)
    gfx.drawRobot(robot.x, robot.y, robot.heading)
    pointCloud = ultraSonic.senseObstacle(robot.x, robot.y, robot.heading)
    robot.avoidObstacle(pointCloud, dt)
    gfx.drawSensorData(pointCloud)

    pygame.display.update()
