#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import os
import csv
import time
import rospy
import matplotlib.pyplot as plt
import matplotlib.colors
from scipy.interpolate import interp1d

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_rs232_v2 import BrickletRS232V2
from tinkerforge.brick_master import BrickMaster

HOST = "localhost"
PORT = 4223
UID_Master = "6SwGJu"
UID_base = "Mad"


# Turns on the LEDs to record data
def turn_on_led():
    # time.sleep(0.1)
    rs232_base.write([ord(c) for c in list("ATLED1=1\r\n")])
    time.sleep(0.1)
    rs232_base.write([ord(c) for c in list("ATLED3=1\r\n")])
    time.sleep(0.1)
    rs232_base.write([ord(c) for c in list("ATLED5=1\r\n")])
    time.sleep(0.1)


# Turns off LEDs after data is collected
def turn_off_led():
    # time.sleep(0.1)
    rs232_base.write([ord(c) for c in list("ATLED1=0\r\n")])
    time.sleep(0.1)
    rs232_base.write([ord(c) for c in list("ATLED3=0\r\n")])
    time.sleep(0.1)
    rs232_base.write([ord(c) for c in list("ATLED5=0\r\n")])
    time.sleep(0.1)


# Checks if the string is float, otherwise throws an error
def is_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


# Receives the data from the Sparkfun triad spectroscopy sensor and reformats the wavelengths into a list of floats.
def process_data(wv):
    data = (''.join(rs232_base.read(150))).split()
    data = [s.strip(',') for s in data]
    data = [float(s) for s in data if is_float(s)]
    data = [x for _, x in sorted(zip(wv, data))]
    return data


# Plots the spectra from the collected data
def plot_spectra(wv_sorted, spectre, count):
    global Y_AXIS_MAX

    # Defines a color map
    norm = matplotlib.colors.Normalize(380, 940)
    colors = [[norm(380), "black"],  # so 410nm looks better
              [norm(405), "indigo"],
              [norm(427), "midnightblue"],
              [norm(435), "darkblue"],
              [norm(460), "blue"],
              [norm(487), "cyan"],
              [norm(510), "green"],
              [norm(520), "darkgreen"],
              [norm(570), "gold"],
              [norm(585), "orange"],
              [norm(610), "orangered"],
              [norm(640), "red"],

              [norm(670), "darkred"],

              [norm(690), "maroon"],
              [norm(720), "black"],
              [norm(940), "black"]]

    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", colors)
    """ END SET COLORS """
    if max(spectre) > Y_AXIS_MAX:
        Y_AXIS_MAX = max(spectre)

    plt.figure(1)

    f = interp1d(wv_sorted, spectre, kind='cubic')
    wv_new = np.linspace(410, 940, 1000)

    plt.rc('font', size=12)

    plt.title('Spectral Response')
    plt.grid(True)
    plt.ylabel('Intensity')
    plt.xlabel('Wavelength (nm)')
    plt.ylim(0, Y_AXIS_MAX*1.1)
    plt.xlim(410, 940)
    plt.plot(wv_new, f(wv_new), label=("Trial " + str(count)))
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.subplots_adjust(left=0.15, bottom=0.15, right=0.72)
    # plt.scatter(wv_new, f(wv_new), c=wv_new, norm=norm, cmap=cmap)
    # sc = ax.scatter(wv_new, f(wv_new), c=wv_new, norm=norm, cmap=cmap)


# Calculate the vegetation indices based on the wavelength intensity data
def calculate_indices(spectre):
    # 810 nm wavelength for NIR
    nir = spectre[14]

    # 680 nm wavelength for red visible
    red = spectre[10]

    # Four different vegetation indices.
    ndvi = (nir - red) / (nir + red)
    rdvi = (nir - red) / (nir + red)**0.5
    msr = ((nir/red)-1) / ((nir/red)+1)**0.5
    savi = (1.5 * (nir - red)) / (nir + red + 0.5)
    print("[NDVI, RDVI, MSR, SAVI]: ", [ndvi, rdvi, msr, savi])
    return [ndvi, rdvi, msr, savi]


if __name__ == "__main__":
    # Connect to TinkerForge stack and initialize ROS node
    ipcon = IPConnection()
    master = BrickMaster(UID_Master, ipcon)
    rs232_base = BrickletRS232V2(UID_base, ipcon)
    rospy.init_node('spectroscopy_base', anonymous=True)
    ipcon.connect(HOST, PORT)

    # Create array of wavelengths (sorted)
    wv = [610, 680, 730, 760, 810, 860, 560, 585, 645, 705, 900, 940, 410, 435, 460, 485, 510, 535]
    wv_sorted = [410, 435, 460, 485, 510, 535, 560, 585, 610, 645, 680, 705, 730, 760, 810, 860, 900, 940]

    # Prompts user for collecting data. Pressing enter turns the LEDs on, collects data, and then turns the LEDs off.
    Y_AXIS_MAX = 0
    counter = 0
    spectre_list = []
    while True:
        user_input = input("Press enter to record spectra, enter e to exit")
        if user_input == "e":
            break
        else:
            counter += 1

            # Turn off blue LED
            print("Turning off blue LED.")
            rs232_base.write([ord(c) for c in list("ATLED0=0\r\n")])
            time.sleep(0.2)

            # Collects data after turning on LEDs
            print("Collecting data...")
            turn_on_led()
            time.sleep(0.1)
            # spectre = collect_data(wv)
            rs232_base.write([ord(c) for c in list("ATCDATA\r\n")])
            time.sleep(0.5)
            turn_off_led()
            time.sleep(0.1)

            # Process the data collected and convert to plot-able format
            print("Data collected, plotting spectra.")
            spectre = process_data(wv)
            spectre_list.append(spectre)
            # print(spectre)

            # Calculate vegetation indices
            vegetation_indices = calculate_indices(spectre)
            plot_spectra(wv_sorted, spectre, counter)

    ipcon.disconnect()

    # Place spectroscopy tool data into .csv file
    folder_path = os.path.dirname(os.path.abspath(__file__))
    print(folder_path)
    file_path = os.path.join(folder_path, 'data_collection_csv/spectroscopy_data.csv')
    print(file_path)
    with open(file_path, 'w', newline='') as my_file:
        csv_writer = csv.writer(my_file, delimiter=',')
        csv_writer.writerow(wv_sorted)

        for row in spectre_list:
            csv_writer.writerow(row)

    # Displays all the plots based on the data.
    plt.show()
