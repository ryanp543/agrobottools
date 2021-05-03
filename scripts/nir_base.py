#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import rospy
import paramiko
import matplotlib.pyplot as plt

port = 22
username = 'pi'
password = 'raspberry'
IP_ADDRESS = '192.168.2.217'

# This function sends a command over SSH to the NeoSpectra Raspberry Pi to run the .sh file that executes the Java code
# for the sensor. The lamp turns on for two seconds, data is collected, and then printed to the terminal, where it is
# read by this script for further analysis.
def capture_data(ssh):
    # Run the agrobottool.sh file on the Raspberry Pi and initialize data collection variables.
    stdin, stdout, stderr = ssh.exec_command('cd Desktop;sh agrobottool.sh')
    data_wavenum_incoming = False
    data_psd_incoming = False
    wavenum = []
    power_spectral_density = []

    # Wait until all of the data is printed to the terminal then break out of the loop
    while True:
        line = stdout.readline()
        if not line:
            break

        # print(line, end="")
        if line.strip('\n') == "2048":
            data_wavenum_incoming = True
        elif data_wavenum_incoming:
            wavenum = line.split()
            data_wavenum_incoming = False
            data_psd_incoming = True
        elif data_psd_incoming:
            power_spectral_density = line.split()
            data_psd_incoming = False

    # Create wavelength and power spectral density list for plotting
    wavelength = [10000000.0 / float(elem) for elem in wavenum]
    power_spectral_density = [float(elem) for elem in power_spectral_density]

    return wavelength, power_spectral_density


# Generates plots using the collected PSD and wavelength data
def generate_plots(wavelength, power_spectral_density, trial_num):
    # print(wavelength)
    # print(power_spectral_density)

    plt.rc('font', size=12)
    plt.figure(1)

    plt.title('Power Spectral Density')
    plt.grid(True)
    plt.ylabel('Power Spectral Density (a.u.)')
    plt.xlabel('Wavelength (nm)')
    # plt.ylim(0, Y_AXIS_MAX*1.1)
    # plt.xlim(410, 940)
    plt.plot(wavelength, power_spectral_density, label=("Trial " + str(trial_num)))
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.subplots_adjust(left=0.15, bottom=0.15, right=0.72)


# Powers down the Raspberry Pi via SSH
def power_down(ssh):
    print("Shutting down now...")
    ssh.exec_command('sudo poweroff')


if __name__ == "__main__":
    # Wait about 1 to 1.5 minutes after powering RaspberryPi before running script to allow it to boot up
    # https://raspberrypi.stackexchange.com/questions/58304/how-to-set-wifi-network-priority
    # Priority 2 on Raspberry Pi is bilab-wifi network (meaning that this is the default network if available)
    # Priority 1 on Raspberry Pi is BilabRover_2.4GHz network
    # The higher the priority number means higher priority

    # Initializes ROS node
    rospy.init_node('nir_base', anonymous=True)
    print("Please allow 60 seconds after turning on power to allow Raspberry Pi to boot up.")
    print("Attempting to connect to Raspberry Pi...")

    # Connects to Raspberry Pi via SSH
    while True:
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(IP_ADDRESS, port, username=username, password=password)
            break
        except TimeoutError:
            print("Failed because the connected party did not properly respond after a period of time or connected host has failed to respond")
            print("Trying again...")
    print("Connected successfully!")

    # User input collecting data interface
    counter = 0
    wv_list = []
    psd_list = []
    while True:
        print("What would you like to do?")
        print("(1) Capture power spectral density")
        print("(2) Exit and plot")
        print("(3) Power down RaspPi, exit, and plot")
        response = input("Please enter command: ")

        # Collect data and add it to the set of datasets
        if response == "1":
            counter += 1
            wv, psd = capture_data(ssh)
            generate_plots(wv, psd, counter)
            print("Data for Trial " + str(counter) + " added to plot.")

            wv_list.append(wv)
            psd_list.append(psd)

        # Just exit and plot, don't shut down Raspberry Pi
        elif response == "2":
            break

        # Exit, shut down Raspberry Pi, and plot data
        elif response == "3":
            power_down(ssh)
            break
        else:
            print("Not a command. Please try again.")

    print("Done. Please allow some time (a handful of seconds) so the Raspberry Pi can shut down.")
    ssh.close()

    # Place spectroscopy tool data into .csv file
    folder_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(folder_path, 'data_collection_csv/nir_spectrometer_data.csv')
    with open(file_path, 'w', newline='') as my_file:
        csv_writer = csv.writer(my_file, delimiter=',')

        for k in range(len(wv_list)):
            csv_writer.writerow(wv_list[k])
            csv_writer.writerow(psd_list[k])

    # Plot all collected data
    plt.show()
