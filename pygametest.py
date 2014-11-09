import pygame
from pygame.locals import *
import os, sys
 
# set SDL to use the dummy NULL video driver, 
#   so it doesn't need a windowing system.
os.environ["SDL_VIDEODRIVER"] = "dummy"

# init pygame
pygame.init()

# create a 1x1 pixel screen, its not used
screen = pygame.display.set_mode((1, 1))

# init the joystick control
pygame.joystick.init()

# how many joysticks are there
print pygame.joystick.get_count()

# get the first joystick
joy = pygame.joystick.Joystick(0)

# init that joystick
joy.init()

running = True
while(running):
    for event in pygame.event.get():
        #thumb sticks, trigger buttons
        if event.type == JOYAXISMOTION:
            
            print event.value, event.axis

        #d pad
        elif event.type == JOYHATMOTION:
            
            print event.value

        #button pressed
        elif event.type == JOYBUTTONDOWN:
            print event.button

        #button released
        elif event.type == JOYBUTTONUP:
            print event.button
