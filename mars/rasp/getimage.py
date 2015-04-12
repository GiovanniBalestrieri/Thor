__author__ = 'robotica'

import pygame
import pygame.camera
import time

pygame.camera.init()
pygame.camera.list_cameras()
cam = pygame.camera.Camera("/dev/video0",(640,480))
cam.start()
for i in range(1):
    img = cam.get_image()
    filename = "filename%d.jpg" % i
    pygame.image.save(img,filename)
    time.sleep(1)
