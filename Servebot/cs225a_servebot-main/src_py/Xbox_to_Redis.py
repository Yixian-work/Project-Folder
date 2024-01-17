import sys
import pygame
import redis 
from collections import defaultdict

redis_cli = redis.Redis(host='localhost', port=6379, db=0)

from pygame.locals import *
pygame.init()
pygame.display.set_caption('Catch Xbox Input')
X = 600
Y = 150
screen = pygame.display.set_mode((X, Y), 0, 32)
clock = pygame.time.Clock()

pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
for joystick in joysticks:
    print(f"INFO: Detected device: {joystick.get_name()}")

verbose = False
debug = False
my_square = pygame.Rect(50, 50, 50, 50)
my_square_color = 0
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
motion = [0, 0]

# Constants
A_BUTTON = 0
B_BUTTON = 1
X_BUTTON = 2
Y_BUTTON = 3
L1_BUTTON = 4
R1_BUTTON = 5
BUTTONS_DICT = defaultdict(int, {A_BUTTON: "A_button",
                                 B_BUTTON: "B_button",
                                 X_BUTTON: "X_button",
                                 Y_BUTTON: "Y_button",
                                 L1_BUTTON: "L1_button",
                                 R1_BUTTON: "R1_button"})

STICK_THRESHOLD = 0.20   # Reject density values below this (prevents stick drifts).
L_STICK_HORIZONTAL = 0
L_STICK_VERTICAL = 1
L_TRIGGER = 2
R_STICK_HORIZONTAL = 3
R_STICK_VERTICAL = 4
R_TRIGGER = 5
AXES_DICT = defaultdict(int, {L_STICK_HORIZONTAL: "L_Stick_Horizontal",
                              L_STICK_VERTICAL: "L_Stick_Vertical",
                              L_TRIGGER: "L_Trigger",
                              R_STICK_HORIZONTAL: "R_Stick_Horizontal",
                              R_STICK_VERTICAL: "R_Stick_Vertical",
                              R_TRIGGER: "R_Trigger"})

white = (255, 255, 255)
green = (0, 255, 0)
blue = (0, 0, 128)
font = pygame.font.Font('freesansbold.ttf', 32)
text = font.render('Focus here to catch Xbox input', True, green, blue)
textRect = text.get_rect()
textRect.center = (X // 2, Y // 2)

def clamp(num, min_value):
    if abs(num) < min_value:
        return 0.0
    else:
        return num

while True:
    screen.fill((0, 0, 0))
    screen.blit(text, textRect)

    print(motion)

    if debug:
        pygame.draw.rect(screen, colors[my_square_color], my_square)
    
    if abs(motion[0]) < 0.1:
        motion[0] = 0
    if abs(motion[1]) < 0.1:
        motion[1] = 0
    my_square.x += motion[0] * 10
    my_square.y += motion[1] * 10
    redis_cli.set(name="Xbox_motion_hor", value=motion[0])
    redis_cli.set(name="Xbox_motion_ver", value=motion[1])

    for event in pygame.event.get():
        if event.type == JOYBUTTONDOWN:
            if verbose: print(event)
            redis_cli.set(name="Xbox_input_button",  value=BUTTONS_DICT[event.button])
            redis_cli.set(name="Xbox_input_density", value=1.0)

            if event.button == 0:
                my_square_color = (my_square_color + 1) % len(colors)

        if event.type == JOYBUTTONUP:
            if verbose: print(event)
            redis_cli.set(name="Xbox_input_button",  value=BUTTONS_DICT[event.button])
            redis_cli.set(name="Xbox_input_density", value=0.0)

        if event.type == JOYAXISMOTION:
            if verbose: print(event)
            if event.axis < 2:
                redis_cli.set(name="Xbox_input_button",  value=AXES_DICT[event.axis])
                redis_cli.set(name="Xbox_input_density", value=clamp(event.value, STICK_THRESHOLD))
                motion[event.axis] = event.value

        if event.type == JOYHATMOTION:
            if verbose: print(event)

        if event.type == JOYDEVICEADDED:
            joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
            for joystick in joysticks:
                print(f"INFO: Added device: {joystick.get_name()}")

        if event.type == JOYDEVICEREMOVED:
            joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                pygame.quit()
                sys.exit()

    pygame.display.update()
    clock.tick(60)
