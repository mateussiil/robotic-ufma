# Based in: https://www.youtube.com/watch?v=pmmUi6DasoM
import numpy as np
import pygame
import math


def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


class Robot:
    def __init__(self, startPos, width):
        self.meterToPixel = 3779.52
        self.width = width

        self.x = startPos[0]
        self.y = startPos[1]
        self.heading = 0

        self.vl = 0.01 * self.meterToPixel  # meters/s
        self.vr = 0.01 * self.meterToPixel

        self.maxSpeed = 0.02 * self.meterToPixel
        self.minSpeed = 0.01 * self.meterToPixel

        self.minObstacleDistance = 100
        self.countDown = 5  # seconds

    def avoidObstacle(self, pointCloud, dt):
        closestObstacle = None
        dist = np.inf

        if len(pointCloud) > 1:
            for point in pointCloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x, self.y], point)
                    closestObstacle = (point, dist)

            if closestObstacle[1] < self.minObstacleDistance and self.countDown > 0:
                self.countDown -= dt
                self.moveBackward()
            else:
                # reset count down
                self.countDown = 5
                # move forward
                self.moveForward()

    def moveBackward(self):
        self.vr = - self.minSpeed
        self.vl = - self.minSpeed/2

    def moveForward(self):
        self.vr = self.minSpeed
        self.vl = self.minSpeed

    def kinematics(self, dt):
        self.x += (((self.vl + self.vr)/2) * math.cos(self.heading)) * dt
        self.y -= (((self.vl + self.vr)/2) * math.sin(self.heading)) * dt
        self.heading += ((self.vr - self.vl) / self.width) * dt

        if self.heading > 2 * math.pi or self.heading < -2 * math.pi:
            self.heading = 0

        self.vr = max(min(self.maxSpeed, self.vr), self.minSpeed)
        self.vl = max(min(self.maxSpeed, self.vl), self.minSpeed)


class Graphics:
    def __init__(self, dimentions, robotImgPath, mapImgPath):
        pygame.init()

        # Colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yellow = (255, 255, 0)

        # MAP
        # Load images
        self.robot = pygame.image.load(robotImgPath)
        self.mapImg = pygame.image.load(mapImgPath)

        # dimentions
        self.height, self.width = dimentions

        # window settings
        pygame.display.set_caption("Obstacle Avoidance")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.mapImg, (0, 0))

    def drawRobot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(
            self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)

    def drawSensorData(self, pointCloud):
        for point in pointCloud:
            pygame.draw.circle(self.map, self.red, point, 3, 0)


class Ultrasonic:
    def __init__(self, sensorRange, map):
        self.sensorRange = sensorRange
        self.mapWidth, self.mapHeight = pygame.display.get_surface().get_size()
        self.map = map

    def senseObstacle(self, x, y, heading):
        obstacles = []
        x1, y1, = x, y
        startAngle = heading - self.sensorRange[1]
        finishAngle = heading + self.sensorRange[1]
        for angle in np.linspace(startAngle, finishAngle, 10, False):
            x2 = x1 + self.sensorRange[0] * math.cos(angle)
            y2 = y1 - self.sensorRange[0] * math.sin(angle)
            for i in range(0, 100):
                u = i / 100
                x = int((x2 * u) + (x1 * (1 - u)))
                y = int((y2 * u) + (y1 * (1 - u)))
                if 0 < x < self.mapWidth and 0 < y < self.mapHeight:
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obstacles.append([x, y])
                        break
        return obstacles
