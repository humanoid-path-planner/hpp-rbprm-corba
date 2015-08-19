#!/usr/bin/env python
import pygame
import time
pygame.init()
pygame.joystick.init()
pygame.event.pump()
if (pygame.joystick.get_count() > 0):
    print "found gamepad! : " + pygame.joystick.Joystick(0).get_name()
    my_joystick = pygame.joystick.Joystick(0)
    my_joystick.init()
    for i in range(100):
        pygame.event.pump()
        time.sleep(0.1)
        x=my_joystick.get_axis(0)
        print x
