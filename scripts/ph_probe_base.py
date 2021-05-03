#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import matplotlib.pyplot as plt
import sys
import queue
import rospy
import os
import csv

HOST = "localhost"
PORT = 4223
UID_R = "Mad"

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_rs232_v2 import BrickletRS232V2


# This callback function collects data from the pH probe tool, which sends readings every second. The data is converted
# to a float from a string of characters.
def cb_base_read(reception):
    global data_queue, DATA, START_DATA_COLLECTION, DATA_LIST, TIME_LIST, START_TIME_DATA, COUNTER
    reception = list(reception)
    # print("new data")
    # print(reception)

    # Links together the characters of a single reading, separated from others by a \r. Also records the time stamp.
    for char in reception:
        if char != '\r':
            DATA += char
        else:
            # print(DATA)
            try:
                if START_DATA_COLLECTION:
                    data_queue.put(float(DATA))
                    COUNTER += 1
                    sys.stdout.write("\rReading: " + str(DATA) + ", Counter: " + str(COUNTER))
                    DATA_LIST.append(float(DATA))
                    TIME_LIST.append(time.time()-START_TIME_DATA)
            except ValueError:
                pass
            DATA = ''


# Generates the plots of the pH over time based on teh collected data.
def generate_plots(t, ph, trial):
    global X_AXIS_MAX
    if len(t) > X_AXIS_MAX:
        X_AXIS_MAX = len(t)

    plt.rc('font', size=12)

    fig1 = plt.figure(1)
    plt.plot(t, ph, label=("Trial " + str(trial)))
    plt.xlabel("Time (s)")
    plt.ylabel("pH")
    plt.title("pH vs. Time")
    plt.xlim([0, X_AXIS_MAX])
    plt.ylim([0, 14])
    plt.yticks(range(0, 15, 1), range(0, 15, 1))
    plt.grid(True, which='both')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.subplots_adjust(left=0.15, bottom=0.15, right=0.72)


if __name__ == "__main__":
    # Connects to TinkerForge stack and initializes ROS node
    ipcon = IPConnection()
    rs232_base = BrickletRS232V2(UID_R, ipcon)
    rospy.init_node('ph_probe_base', anonymous=True)

    ipcon.connect(HOST, PORT)

    # Initializes data collection variables
    START_DATA_COLLECTION = False
    DATA = ''
    X_AXIS_MAX = 0
    data_queue = queue.Queue()

    # Configures the RS232 bricklet for communication
    rs232_base.register_callback(rs232_base.CALLBACK_READ, cb_base_read)
    rs232_base.enable_read_callback()
    rs232_base.set_configuration(9600, 0, 1, 8, 0)

    # Initializes data lists
    DATA_LIST = []
    TIME_LIST = []
    COUNTER = 0
    START_TIME_DATA = time.time()
    time.sleep(1)

    # Waits for the probe to settle to around a neutral number before data collection (though this is probably
    # unnecessary).
    print("Waiting for probe to settle below 8.5...")
    START_DATA_COLLECTION = True
    while True:
        reading = data_queue.get(True)
        if reading > 8.5:
            pass
        else:
            break
    START_DATA_COLLECTION = False

    print("\nReady to collect data!\n")
    test_number = 0

    time_list_total = []
    data_list_total = []

    # On user input, data collection begins. When the user presses enter a second time, data collection stops.
    while True:
        DATA_LIST = []
        TIME_LIST = []
        COUNTER = 0
        user_input = input("Press enter to start recording or e to exit\n")

        # Entering e exits the loop
        if user_input == 'e':
            break

        # Otherwise start collecting data and press enter to exit
        else:
            print("Starting data collection...")
            START_TIME_DATA = time.time()
            DATA_LIST = []
            TIME_LIST = []
            COUNTER = 0
            START_DATA_COLLECTION = True
            # rs232_base.enable_read_callback()

            input("Press enter to stop recording\n")
            START_DATA_COLLECTION = False

            # print(DATA_LIST)
            # print(len(DATA_LIST))
            # print(TIME_LIST)
            # print(len(TIME_LIST))

            # Calculates the average and standard deviation of the data
            mean = sum(DATA_LIST)/len(DATA_LIST)
            variance = sum([((x - mean) ** 2) for x in DATA_LIST]) / len(DATA_LIST)
            std = variance ** 0.5
            print("\nAverage: " + str(mean))
            print("Standard deviation: " + str(std))
            print("Adding to plot...")
            test_number += 1
            generate_plots(TIME_LIST, DATA_LIST, test_number)

            time_list_total.append(TIME_LIST)
            data_list_total.append(DATA_LIST)

    ipcon.disconnect()

    # Export data to .csv file
    folder_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(folder_path, 'data_collection_csv/ph_data.csv')
    with open(file_path, 'w', newline='') as my_file:
        csv_writer = csv.writer(my_file, delimiter=',')

        for k in range(0, len(time_list_total)):
            csv_writer.writerow(time_list_total[k])
            csv_writer.writerow(data_list_total[k])

    print("CSV file written. Displaying initial plot.")

    # Display data in a plot.
    plt.show()
