#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import csv
import os
import io
import pygame
import rospy
import sys
import queue
import numpy as np

HOST = "localhost"
PORT = 4223
UID_R = "Mad"

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_rs232_v2 import BrickletRS232V2

# Initialize global variables
frame_queue = queue.Queue()
IMAGE_DATA = []
MIN_TEMP_IN_CELSIUS = 25    # TO BE CHANGED BY USER
MAX_TEMP_IN_CELSIUS = 40    # TO BE CHANGED BY USER
MIN_TEMP_STRING = "{0:0=3d}".format(MIN_TEMP_IN_CELSIUS)
MAX_TEMP_STRING = "{0:0=3d}".format(MAX_TEMP_IN_CELSIUS)


# Calculates the temperature based on the grayscale pixel of the image
def map_g_to_temp(g):
    return ((g * (MAX_TEMP_IN_CELSIUS - MIN_TEMP_IN_CELSIUS)) / 255.0) + MIN_TEMP_IN_CELSIUS


# Generates an array of temperatures for each pixel in the grayscale image
def get_temperature_array(image, screen_w, screen_h):
    thermal_array = pygame.surfarray.array3d(image)
    avgs = [[(r * 0.298 + g * 0.587 + b * 0.114) for (r, g, b) in col] for col in thermal_array]
    array2d = np.array([[[avg, avg, avg] for avg in col] for col in avgs])
    array2d = np.reshape(np.delete(array2d, [1, 2], axis=2), (screen_w, screen_h))

    # data_string = np.frombuffer(data_string, dtype=np.uint8)
    temp_array = np.array([[map_g_to_temp(pixel) for pixel in row] for row in array2d])

    return temp_array


# Displays a picture in the Pygame browser
def disp_picture(pic_data):
    # Initialize browser settings
    pygame.init()
    screen_w = 640
    screen_h = 480
    screen = pygame.display.set_mode((screen_w, screen_h), flags=pygame.RESIZABLE)
    pygame.display.set_caption("Frame Buffer")
    clock = pygame.time.Clock()

    # Convert data to a string and then load the thermal image onto the Pygame browser
    pic_data_string = bytes(''.join(pic_data), 'raw_unicode_escape')
    thermal_image = pygame.transform.scale(pygame.image.load(io.BytesIO(pic_data_string), "jpg"), (screen_w, screen_h))

    # Get the full array of temperatures and identify the maximum temperature (and which pixel it is)
    temp_array = get_temperature_array(thermal_image, screen_w, screen_h)
    max_temp = np.amax(temp_array)
    max_index = np.unravel_index(temp_array.argmax(), temp_array.shape)
    # print(max_temp)
    # print(max_index)

    # Display the maximum temperature in a box on the display frame.
    font = pygame.font.Font('freesansbold.ttf', 32)
    text = font.render("{:.3f}".format(max_temp) + " C", True, (255, 0, 0), (255, 255, 255))
    textRect = text.get_rect()
    textRect.center = (520, 420)

    # Exports temperature array to a .csv file
    print("Generating CSV file...")
    folder_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(folder_path, 'data_collection_csv/thermal_data.csv')
    with open(file_path, 'w', newline='') as myfile:
        csvwriter = csv.writer(myfile, delimiter=',')

        for row in temp_array:
            csvwriter.writerow(row)
    print("CSV file generated.")

    # Plots circle around the hottest pixel
    while True:
        try:
            screen.blit(thermal_image, (0, 0))
            screen.blit(text, textRect)
            pygame.draw.circle(thermal_image, (255, 0, 0), max_index, 10, 3)
            pygame.display.update()
        except pygame.error:
            break

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                # quit()
                break


# Displays a live video of the thermal camera grayscale image
def disp_video():
    # Initialize browser settings
    pygame.init()
    screen_w = 640
    screen_h = 480
    screen = pygame.display.set_mode((screen_w, screen_h), flags=pygame.RESIZABLE)
    pygame.display.set_caption("Frame Buffer")
    clock = pygame.time.Clock()

    # Plots the current image returned by the thermal camera tool
    while True:
        sys.stdout.flush()

        try:
            frame = list(frame_queue.get(False))
            frame_data_string = bytes(''.join(frame), 'raw_unicode_escape')

            screen.blit(pygame.transform.scale(pygame.image.load(io.BytesIO(frame_data_string), "jpg"), (screen_w, screen_h)), (0, 0))
            pygame.display.update()
            clock.tick()
        except queue.Empty:
            pass
        except pygame.error:
            break

        # print(clock.get_fps())

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                # quit()
                rs232_base.write('END000000')
                break


# Callback function collecting data containing a snapshot from the Lepton.
def cb_read_picture(reception):
    global IMAGE_DATA
    IMAGE_DATA.extend(reception)
    if IMAGE_DATA[-9:] == ['_', '_', '_', 'e', 'n', 'd', '_', '_', '_']:
        pic_data = IMAGE_DATA[11:-9]
        IMAGE_DATA = []
        # print(pic_data)
        # print(len(pic_data))
        disp_picture(pic_data)


# Callback function collecting data containing a snapshot and then putting that snapshot into a queue
def cb_read_video(reception):
    global frame_queue, IMAGE_DATA
    IMAGE_DATA.extend(reception)
    if IMAGE_DATA[-9:] == ['_', '_', '_', 'e', 'n', 'd', '_', '_', '_']:
        frame_data = IMAGE_DATA[11:-9]
        IMAGE_DATA = []
        # print(frame_data)
        # print(len(frame_data))
        frame_queue.put(frame_data)


if __name__ == "__main__":
    # Connect to TinkerForge stack and initialize ROS node
    ipcon = IPConnection()
    rs232_base = BrickletRS232V2(UID_R, ipcon)
    rospy.init_node('thermal_camera_base', anonymous=True)
    ipcon.connect(HOST, PORT)

    # Configure RS232 bricklet settings.
    rs232_base.set_configuration(921600, 0, 1, 8, 0)

    # Prompts user to select video or picture option and then enables the correct callback function before writing the
    # command to the tool.
    running = True
    while running:
        print("1) For picture")
        print("2) For video")
        x = input("Enter a number or 'e' to exit.\n")
        if x == 'e':
            running = False
        elif x == '1':
            rs232_base.register_callback(rs232_base.CALLBACK_READ, cb_read_picture)
            rs232_base.enable_read_callback()
            command = "PIC" + MIN_TEMP_STRING + MAX_TEMP_STRING
            print("Command sent: ", command)
            rs232_base.write(command)
        elif x == '2':
            print("Asking for video")
            rs232_base.register_callback(rs232_base.CALLBACK_READ, cb_read_video)
            rs232_base.enable_read_callback()
            command = "VID" + MIN_TEMP_STRING + MAX_TEMP_STRING
            rs232_base.write(command)
            disp_video()

    ipcon.disconnect()

