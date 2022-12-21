import math
import pygame

pygame.init()

screen_x = 640/2
screen_y = 480/2
screen_size = (screen_x, screen_y)

# importante pra botar a imagem o eixo no centro
middle_x = screen_x/2
middle_y = screen_y/2

# Create a screen to draw on
screen = pygame.display.set_mode(screen_size)

# Set the title of the window
# pygame.display.set_caption('Joint System Simulation')

# Set the background color
bg_color = (255, 255, 255)

font = pygame.font.Font(None, 36)
text_color = (0, 0, 0)

# Set the joint lengths
L1 = 40
L2 = 40

# Set the joint radius
joint_radius = 3

# Set the link width
link_width = 5

# Set the joint and link colors
joint_color = (0, 0, 0)
initial_joint_color = (0,255,0)
link_color_1 = (255, 0, 0)
link_color_2 = (0, 255, 0)

# Set the initial coordinates and orientation of the endpoint
x = -26
y = 39
phi = 30

# Set the joint angles to the initial values
theta1 = 0
theta2 = 0

# Set the frame rate
frame_rate = 60
clock = pygame.time.Clock()

import math
import pygame


class JointSystem:
    def __init__(self, L1, L2):
        self.L1 = L1
        self.L2 = L2

    def degToRad(self, degrees):
        return degrees * math.pi / 180

    def radToDeg(self, radians):
        return radians * 180 / math.pi

    def inverseKinematics(self, x, y):
        x1 = x
        y1 = y

        cosTheta2 = (pow(x1, 2) + pow(y1, 2) - pow(self.L1, 2) - pow(self.L2, 2)) / (2 * L1 * L2);
        sinTheta2 = math.sqrt(1 - pow(cosTheta2, 2))
        theta2 = math.atan2(sinTheta2, cosTheta2)

        k1 = L1 + L2*cosTheta2
        k2 = L2 * sinTheta2

        theta1 = math.atan2(k1*y1-k2*x1, k1*x1+k2*y1)

        return theta1, theta2


def show_mouse_pos(pos):
    mouse_x = pos[0]
    mouse_y = pos[1]
    distancia = math.sqrt(pow(middle_x - mouse_x, 2) + pow(middle_y - mouse_y, 2))
    print("x", middle_x - mouse_x)
    print("y", middle_y - mouse_y)

joint_system = JointSystem(L1, L2)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            pygame_mouse_pos = pygame.mouse.get_pos()
            show_mouse_pos(pygame_mouse_pos)

    theta1, theta2 = joint_system.inverseKinematics(x, y)

    screen.fill(bg_color)

    # A primeira junta é o centro
    joint1_pos = (middle_x, middle_y)

    # Calculando o ponto x,y da segunda junta
    joint2_x = middle_x + L1 * math.cos(theta1)
    joint2_y = middle_y - L1 * math.sin(theta1)
    joint2_pos = (joint2_x, joint2_y)
    
    # Calculando onde fica o ponto final
    endpoint_x = joint2_x + L2 * math.cos(theta1 + theta2)
    endpoint_y = joint2_y - L2 * math.sin(theta1 + theta2)
    endpoint_pos = (endpoint_x, endpoint_y)

    # links
    pygame.draw.line(screen, link_color_1, joint1_pos, joint2_pos, link_width)
    pygame.draw.line(screen, link_color_2, joint2_pos, endpoint_pos, link_width)

    # Juntas
    pygame.draw.circle(screen, initial_joint_color, joint1_pos, joint_radius)
    pygame.draw.circle(screen, joint_color, joint2_pos, joint_radius)
    pygame.draw.circle(screen, joint_color, endpoint_pos, joint_radius)

    pygame.display.flip()

    clock.tick(frame_rate)

pygame.quit()