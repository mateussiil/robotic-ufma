# Codigo baseado em https://www.youtube.com/watch?v=zHboXMY45YU&list=WL&index=1

import pygame
import math
import numpy as np
from numpy import array, zeros, diag, diagflat, dot
import matplotlib.pyplot as plt


class Env:  # Environment class
    def __init__(self, dimentions):
        # Colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yel = (255, 255, 0)

        # Dimensions
        self.height = dimentions[0]
        self.width = dimentions[1]

        # Window settings
        pygame.display.set_caption("Mobile robot")
        self.map = pygame.display.set_mode((self.width, self.height))

        # Text configuration
        self.font = pygame.font.Font('freesansbold.ttf', 20)
        self.text = self.font.render('default', True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dimentions[1]-600, dimentions[0]-100)

        # Trail
        self.trail_set = []

    def write_info(self, x, y, velocity, theta):
        Pose = f"x = {x}; y = {y}; v = {velocity}; theta = {int(math.degrees(theta))}"
        self.text = self.font.render(Pose, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)

    def trail(self, pos):
        for i in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.map, self.yel, (self.trail_set[i][0], self.trail_set[i][1]), (
                self.trail_set[i+1][0], self.trail_set[i+1][1]))

        if self.trail_set.__sizeof__() > 10000:  # Numbe of elements on the trail
            self.trail_set.pop(0)

        self.trail_set.append(pos)

    def robot_frame(self, pos, rotation):
        n = 80
        centerx, centery = pos
        x_axis = (centerx + n * math.cos(-rotation),
                  centery + n * math.sin(-rotation))
        y_axis = (centerx + n * math.cos(-rotation + math.pi/2),
                  centery + n * math.sin(-rotation + math.pi/2))
        pygame.draw.line(self.map, self.red, (centerx, centery),
                         x_axis, 3)  # X axis in red
        pygame.draw.line(self.map, self.green,
                         (centerx, centery), y_axis, 3)  # Y axis in green


class Odometry:
    def __init__(self):
        self.vd = 0.1
        self.vtheta = 0.001
        self.sigma_d = 0.0001
        self.sigma_theta = 0.0001


class Robot:
    def __init__(self, startPos, desiredPos, robotImg, width, odometry):
      # Initial conditions
        self.w = width
        self.x = startPos[0]
        self.y = startPos[1]
        self.xw, self.yw = desiredPos
        self.theta = 0
        self.gamma = 0
        self.delta_d = 1.5
        self.delta_theta = 0.001
        self.v = 0.002
        self.odometry = odometry
        self.positions = [[self.x, self.y, self.theta]]
        self.errorPositions = [[self.x, self.y, self.theta]]
        self.kalmanPositions = [[self.x, self.y, self.theta]]

        # Robot
        self.img = pygame.image.load(robotImg)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def route(self):
        if (self.x < 1000 and self.y < 200):
            self.delta_theta = 0
        if (self.x > 1000 and self.y < 100):
            self.delta_theta = 5
        if (self.x < 1000 and self.y > 200):
            self.delta_theta = 25
        if (self.x > 1000 and self.y > 100):
            self.delta_theta = 0

    def estimatePose(self):
        # Estimating Pose
        x_last_v, y_last_v, theta_last_v = self.errorPositions[-1]

        errorX = x_last_v + (self.delta_d + self.odometry.vd) * \
            math.cos(self.theta)
        errorY = y_last_v + (self.delta_d + self.odometry.vd) * \
            math.sin(self.theta)
        errorTheta = theta_last_v + self.delta_theta + self.odometry.vtheta

        error_pose = [errorX, errorY, errorTheta]

        self.errorPositions.append(error_pose)

        v = [
            [self.odometry.sigma_d**2, 0],
            [0, self.odometry.sigma_theta**2]
        ]

        fx = [
            [1, 0, -self.delta_d*math.sin(self.odometry.vtheta)],
            [0, 1, self.delta_d*math.cos(self.odometry.vtheta)],
            [0, 0, 1]
        ]

        fx_t = np.array(fx).transpose()

        fv = [
            [math.sin(self.odometry.vtheta), 0],
            [math.cos(self.odometry.vtheta), 0],
            [0, 1]
        ]

        fv_t = np.array(fv).transpose()

        estimated_pose = np.add(
            np.dot(np.dot(fx, error_pose), fx_t),
            np.dot(np.dot(fv, v), fv_t)
        )

        self.kalmanPositions.append(estimated_pose[1])

    def move(self, event=None):
        # Mathematical differential-drive model

        x_last, y_last, theta_last = self.positions[-1]

        # self.route()
        self.x = x_last + self.delta_d*math.cos(self.theta)
        self.y = y_last + self.delta_d*math.sin(self.theta)
        self.theta = theta_last + self.delta_theta

        if (self.theta > 2*math.pi or self.theta < -2*math.pi):
            self.theta = 0

        real_pose = [self.x, self.y, self.theta]
        self.positions.append(real_pose)

        self.estimatePose()

        #Change in orientation
        # Rotate image 'theta' with a scale operation of 1 - no change in size
        self.rotated = pygame.transform.rotozoom(
            self.img, math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))


# Initialisation
pygame.init()

# Dimensions
dims = (600, 1200)

# Status
running = True

# Environment
environment = Env(dims)

# Robot
start_pos = (300, 300)
desired_pos = (0, 0)
img_add = "robo.png"
# robot_width = 0.01*3779.52 # 1cm
robot_width = 1  # 8 pixels
robot = Robot(start_pos, desired_pos, img_add, robot_width, Odometry())
# dt
dt = 0
lasttime = pygame.time.get_ticks()

# Simulation loop
while running:
    # Verify events

    # Time change
    # Current minus last time # Time in seconds
    dt = (pygame.time.get_ticks() - lasttime)/1000
    lasttime = pygame.time.get_ticks()  # Update last time

    for event in pygame.event.get():
        if event.type == pygame.QUIT:  # Quit the window
            running = False
        robot.move(event)

    # Update
    pygame.display.update()
    environment.map.fill(environment.black)
    robot.move()

    environment.write_info(int(robot.x), int(
        robot.y), round(robot.v, 2), robot.theta)

    robot.draw(environment.map)
    environment.robot_frame((robot.x, robot.y), robot.theta)

    environment.trail((robot.x, robot.y))

x_real = []
y_real = []
x_error = []
y_error = []
x_filterKalman = []
y_filterKalman = []

for value in robot.positions:
    x_real.append(value[0])
    y_real.append(value[1])

for value in robot.errorPositions:
    x_error.append(value[0])
    y_error.append(value[1])

for value in robot.kalmanPositions:
    x_filterKalman.append(value[0])
    y_filterKalman.append(value[1])

plt.figure()
plt.grid()
plt.plot(x_real, y_real, 'g', label="Robot")
plt.plot(x_filterKalman, y_filterKalman, 'b--', label="Kalman Filter")
plt.legend(loc='upper center')
plt.show()
