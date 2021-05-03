#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import os
import csv
import queue
import rospy
import struct
import math
import numpy as np
import matplotlib.pyplot as plt

HOST = "localhost"
PORT = 4223
UID_R = "Mad"

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_rs232_v2 import BrickletRS232V2

DATA_PACKET = []

# Gain = Output Excitation Voltage * PGA Gain * Gain Setting Resistor Impedance / Z unknown
# Note: The gain applied (calculated using equation above) must be <3V but >0.5V. Due to limitations in voltage range
# and PGA gain, it's tough to calibrate gain factor curve for 1K to 10K ohms.
# Too high applied gain -> saturation of ADC. Too low, and 12 bit ADC can't measure any changes.
# GF_POLYNOMIAL = [5.69679755e-18, -7.45432105e-15, 1.01276224e-06]    # 2000 mV P-P (100R ref resistor) PWR to SEL, x5
# GF_POLYNOMIAL = [5.79940851e-18, -1.56939859e-13, 9.97159849e-07]    # 2000 mV P-P (1K ref resistor) PWR to SEL, x5
# GF_POLYNOMIAL = [1.47230057e-19, -6.88302825e-15, 5.70095652e-09]    # 400 mV P-P (10K ref resistor) GND to SEL, x1
# GF_POLYNOMIAL = [2.78221242e-20, -4.25809567e-17, 2.20112151e-09]    # 1000 mV P-P (100K ref resistor) GND to SEL, x1

# Calibration resistor
CALIBRATION_R = 1000  # in OHMS. Use resistor + 2*50 Ohms for the two SMA connectors
data_queue = queue.Queue()


class Command:
    def __init__(self, cal_flag, vol_rng, begin_freq, freq_step, nsamples, rep_flag, pga_gain):
        # CHECK IF STRING IS CORRECT NUMBER OF CHARACTERS
        if not all(isinstance(item, str) for item in [cal_flag, vol_rng, begin_freq, freq_step, nsamples, rep_flag, pga_gain]):
            raise TypeError("All arguments of class Command must be strings!")

        if len(cal_flag) != 1 or len(vol_rng) != 4 or len(begin_freq) != 6 or len(freq_step) != 6 or len(nsamples) != 3 \
                or len(rep_flag) != 1 or len(pga_gain) != 1:
            raise ValueError("One of the desired values is not the correct length of characters!")

        # Initialize class variables
        self.calibration_flag = cal_flag    # 1 for calibration, 0 for measurement
        self.voltage_range = vol_rng        # Voltage range in mV
        self.start_freq = begin_freq        # Start frequency
        self.freq_inc = freq_step           # Frequency increment
        self.num_samples = nsamples         # Number of samples, max 511
        self.repeat_flag = rep_flag         # 0 for false, 1 for true
        self.gain = pga_gain                # PGA gain value (1 or 5)

        # Check if the values are valid for operation, throw error if otherwise
        if int(vol_rng) not in [200, 400, 1000, 2000]:
            raise ValueError("Unable to set voltage range. Make sure voltage range is 200, 400, 1000 or 2000 mV.")
        if int(begin_freq) < 1000 or int(begin_freq) > 100000:
            raise ValueError("Start frequency cannot be below 1000 Hz or above 100000 Hz.")
        if int(freq_step) > 99000:
            raise ValueError("Frequency step cannot be greater than 99000 Hz.")
        if int(nsamples) > 511:
            raise ValueError("You cannot have more than 511 samples.")
        if int(pga_gain) not in [1, 5]:
            raise ValueError("PGA gain must be 1 or 5.")
        if (int(begin_freq) + (int(freq_step) * (int(nsamples) - 1))) > 100000:
            raise ValueError("One of the samples is taken at a frequency above 100000 Hz.")

        # If no errors were thrown from if-statements above, set class variables
        self.voltage_range_int = int(vol_rng)
        self.start_freq_int = int(begin_freq)
        self.freq_inc_int = int(freq_step)
        self.num_samples_int = int(nsamples)
        self.gain_int = int(pga_gain)

    def get_command(self):
        # This function returns the command that was sent to it
        return self.calibration_flag + self.voltage_range + self.start_freq + self.freq_inc + self.num_samples + \
               self.repeat_flag + self.gain

    def get_frequency_range(self):
        # This function returns the upper and lower bounds on the frequency range used for analysis
        return range(self.start_freq_int, self.start_freq_int + ((self.num_samples_int - 1) * self.freq_inc_int),
                     self.freq_inc_int)


def cb_read(reception):
    # This callback function receives the data from the Teensy in byte format and then converts it all into an array of
    # data values from which the magnitude can be calculated
    global DATA_PACKET
    DATA_PACKET = DATA_PACKET + list(reception)
    if (len(DATA_PACKET) >= 4) and (DATA_PACKET[-4:] == ['\x00', '\x01', '\x02', '\x03']):
        # Convert all the characters to strings of the hex values
        data_values = []
        data = [hex(ord(data_byte)) for data_byte in DATA_PACKET[:-4]]

        for k in range(0, len(data), 3):
            if data[k] == '0x0':
                sign = '0000'
            elif data[k] == '0x1':
                sign = 'FFFF'
            else:
                sign = 0
            hex_string = sign + ''.join([format(int(c, 16), '02X') for c in [data[k+1], data[k+2]]])
            data_values.append(struct.unpack('>i', bytes.fromhex(hex_string))[0])

        data_queue.put(data_values)
        DATA_PACKET = []


def calculate_magnitude(real_data, imag_data):
    # This function calculates the magnitude from the real and imaginary data received from the Teensy/Pmod-IA.
    return [math.sqrt(real_data[k] ** 2 + imag_data[k] ** 2) for k in range(len(real_data))][1:]


def calculate_gf(magnitude):
    # This function calculates the gain factor from the measured magnitude and the calibration resistor.
    return [(1/CALIBRATION_R)/magnitude[k] for k in range(len(magnitude))]


def calculate_gf_polynomial(freq):
    # This function calculates the gain factor using the polynomial model that was generated during calibration.
    global GF_POLYNOMIAL
    gain_factor = (GF_POLYNOMIAL[0] * (freq**2)) + (GF_POLYNOMIAL[1] * freq) + GF_POLYNOMIAL[2]
    return gain_factor


def calculate_impedance(magnitude, gain_factor):
    # This function calculates the impedance given the magnitude and the gain factor (equation from Pmod-A documents).
    return [1/(magnitude[k] * gain_factor[k]) for k in range(len(magnitude))]


def calculate_capacitance(impedance, frequencies):
    # This function calculates the capacitance given the impedance and frequencies.
    return [1/(2*math.pi*frequencies[k]*impedance[k]) for k in range(len(impedance))]


def extract_data(cmd):
    # Collect calibration data from queue
    data = data_queue.get(True)
    real_data = data[0::2]
    imag_data = data[1::2]

    # Calculate gain factors (based on polynomial) and magnitudes to then calculate impedance
    frequencies = cmd.get_frequency_range()
    gain_factors = [calculate_gf_polynomial(f) for f in frequencies]
    magnitude = calculate_magnitude(real_data, imag_data)
    impedance = calculate_impedance(magnitude, gain_factors)
    print("Average measured impedance (Ohms): ", sum(impedance[31:])/len(impedance[31:]))

    # Calculate capacitance
    capacitance = calculate_capacitance(impedance, frequencies)
    avg_capacitance = sum(capacitance[31:]) / len(capacitance[31:])
    print("Average measured capacitance (F): ", avg_capacitance)

    return frequencies, magnitude, impedance, capacitance


def command_calibrate():
    # This function is called when the gain factor polynomial model needs to be calibrated. The command specified in
    # the cmd variable is sent via the rs232 bricklet.
    # Currently set for 10K reference resistor
    cmd = Command('0', '0400', '001000', '000198', '501', '0', '1')
    print("Command: ", cmd.get_command())
    rs232_base.write(cmd.get_command())
    return cmd


def command_measure(cmd):
    # This function writes the desired command cmd to the impedance analyzer tool via the RS232/TOSLink assembly.
    print("Command: ", cmd.get_command())
    rs232_base.write(cmd.get_command())


def generate_gain_factor_plot(cmd):
    # This function plots the gain factor calibration curve and fits a second-degree polynomial model to the data.;
    global GF_POLYNOMIAL

    # Collect calibration data from queue
    calibration_data = data_queue.get(True)
    real_data = calibration_data[0::2]
    imag_data = calibration_data[1::2]

    # Calculate list of magnitudes and gains
    magnitude = calculate_magnitude(real_data, imag_data)
    gain_factor_cal = calculate_gf(magnitude)
    print(gain_factor_cal)
    print("Average gain factor: ", sum(gain_factor_cal[31:]) / len(gain_factor_cal[31:]))

    # Calculate list of frequencies
    frequencies = cmd.get_frequency_range()
    print(frequencies)

    # Calculate best fit polynomial (least squares). Only values after element 31 are used because it looks like
    # saturation is beginning to occur before element 31.
    polynomials = np.polyfit(frequencies[50:], gain_factor_cal[50:], 2)
    print(polynomials)
    GF_POLYNOMIAL = polynomials

    # Generate polynomial curve fit line for plotting
    print(type(GF_POLYNOMIAL))
    curve_fit = []
    for hz in frequencies[31:]:
        curve_fit.append(GF_POLYNOMIAL[0]*(hz**2) + GF_POLYNOMIAL[1]*hz + GF_POLYNOMIAL[2])

    # Generate gain factor plot
    plt.rc('font', size=12)
    plt.scatter(frequencies[31:], gain_factor_cal[31:], label='Data')
    plt.plot(frequencies[31:], curve_fit, color='r', label='Curve Fit')
    plt.title("Calibration Curve")
    plt.xlabel("Frequencies (kHz)")
    plt.ylabel("Gain Factor")
    axes = plt.gca()
    axes.set_ylim([0.95*min(gain_factor_cal[31:]), 1.05*max(gain_factor_cal[31:])])
    axes.set_xlim(0, 120000)
    axes.set_xticklabels([0, 20, 40, 60, 80, 100, 120])
    plt.grid()
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.subplots_adjust(left=0.15, bottom=0.15, right=0.71)
    plt.show()


def generate_plots(frequencies, magnitude, impedance, capacitance, series_label):
    # This function plots the magnitude, impedance and capacitance of a data set with respect to the frequencies.
    plt.rc('font', size=12)

    # Plotting magnitude vs frequency
    fig1 = plt.figure(1)
    plt.loglog(frequencies[31:], magnitude[31:], label=series_label)
    plt.xlabel("Frequencies (Hz)")
    plt.ylabel("Magnitude")
    plt.title("Magnitude vs Frequency")
    plt.xlim([7000, 100000])
    # ax2.set_ylim([100,21000000])
    plt.grid(True, which='both')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.subplots_adjust(left=0.15, bottom=0.15, right=0.72)

    # Plotting impedance vs frequency
    fig2 = plt.figure(2)
    plt.loglog(frequencies[31:], impedance[31:], label=series_label)
    plt.xlabel("Frequencies (Hz)")
    plt.ylabel("Impedance (Ohms)")
    plt.title("Impedance vs Frequency")
    plt.xlim([7000, 100000])
    plt.ylim([100, 1000000])
    plt.grid(True, which='both')
    plt.legend(bbox_to_anchor=(1.05, 1), title='Moisture (%)', loc='upper left')
    plt.subplots_adjust(left=0.15, bottom=0.15, right=0.71)

    # Plotting capacitance vs frequency
    fig3 = plt.figure(3)
    plt.loglog(frequencies[31:], capacitance[31:], label=series_label)
    plt.xlabel("Frequencies (Hz)")
    plt.ylabel("Capacitance (F)")
    plt.title("Capacitance vs Frequency")
    plt.xlim([7000, 100000])
    # plt.ylim([avg_capacitance/10, avg_capacitance*10])
    plt.grid(True, which='both')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.subplots_adjust(left=0.15, bottom=0.15, right=0.72)


if __name__ == "__main__":
    # Initialize connection to TinkerForge system
    ipcon = IPConnection()
    rs232_base = BrickletRS232V2(UID_R, ipcon)

    # Initialize ROS node
    rospy.init_node('impedance_probe_base', anonymous=True)

    # Connect to the TinkerForge system
    ipcon.connect(HOST, PORT)

    # Set RS232 configurations to communicate with attached TOSLink PCB
    rs232_base.set_configuration(115200, 0, 1, 8, 0)
    rs232_base.register_callback(rs232_base.CALLBACK_READ, cb_read)
    rs232_base.enable_read_callback()

    csv_list = []

    # Collecting data
    while True:
        print("1) Calibrate")
        print("2) Take impedance measurements")
        print("3) Exit")
        user_input = input("What would you like to do?\n")

        # Sends command to calibrate, note the value of resistance between probes (i.e. CALIBRATION_R). Generates
        # polynomial model for gain factors, which are then hardcoded below.
        if user_input == '1':
            command = command_calibrate()
            generate_gain_factor_plot(command)

        # Sends a command depending on the expected range of resistances and then collects the received data
        elif user_input == '2':
            print("a) 100 to 10K Ohms (~1 to 10 nF)")
            print("b) 10K to 100K Ohms")
            print("c) 100K to 1M Ohms")
            user_input = input("Which range of resistances are you expecting?\n")

            # command: (cal_flag, vol_rng, begin_freq, freq_step, nsamples, rep_flag, pga_gain)
            # Note that the polynomial model GF_POLYNOMIAL is hardcoded with values obtained using command_calibrate()
            if user_input == 'a':
                command = Command('0', '2000', '001000', '000198', '501', '0', '5')
                GF_POLYNOMIAL = [5.69679755e-18, -7.45432105e-15, 1.01276224e-06]
            elif user_input == 'b':
                command = Command('0', '0400', '001000', '000198', '501', '0', '1')
                GF_POLYNOMIAL = [1.47230057e-19, -6.88302825e-15, 5.70095652e-09]
            elif user_input == 'c':
                command = Command('0', '1000', '001000', '000198', '501', '0', '1')
                GF_POLYNOMIAL = [2.78221242e-20, -4.25809567e-17, 2.20112151e-09]

            try:
                # Send command, collect data
                command_measure(command)
                freq, mag, imp, cap = extract_data(command)

                # Plot data
                res_input = input("Add legend label for data (i.e. resistance of resistor, soil moisture, etc.): ")
                generate_plots(freq, mag, imp, cap, res_input)

                # Add to list for .csv file
                csv_list.append(imp)
            except NameError:
                print("Not an option, try again.")

        # Breaks from user input loop
        elif user_input == '3':
            break

        # Catches incorrect user inputs
        else:
            print("Not an option.")

    # Disconnect from TinkerForge stack
    ipcon.disconnect()

    # Place impedance data into .csv file
    folder_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(folder_path, 'data_collection_csv/impedance_data.csv')
    with open(file_path, 'w', newline='') as my_file:
        csv_writer = csv.writer(my_file, delimiter=',')
        csv_writer.writerow(freq)

        for row in csv_list:
            csv_writer.writerow(row)

    # Show all plots from all test trials
    plt.show()
