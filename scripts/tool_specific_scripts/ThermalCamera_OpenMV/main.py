# Untitled - By: rpooo - Fri Jul 31 2020

import sensor, image, time, math
from pyb import UART, LED

def map_g_to_temp(g):
    return ((g * (max_temp_in_celsius - min_temp_in_celsius)) / 255.0) + min_temp_in_celsius


def take_picture():
    LED(2).off()

    start_message = '___start___'
    end_message = '___end___'
    img = sensor.snapshot()
    zeros = bytes(160*120)

    while(True):
        img = sensor.snapshot()
        img_b = img.bytearray()

        if img_b != zeros:
            uart.write(start_message)
            uart.write(img.compress(quality=90))
            uart.write(end_message)
            break


def take_video():
    LED(2).off()

    start_message = '___start___'
    end_message = '___end___'

    while(True):
        message = uart.read(9)

        if message is None:
            img = sensor.snapshot()

            #img_b = img.bytearray()
            #print(img_b)
            #print(len(img_b))

            time.sleep(100)

            uart.write(start_message)
            uart.write(img.compress(quality=90))
            uart.write(end_message)
        else:
            break


# Setup UART
uart = UART(3, 921600, timeout_char=1000)
uart.init(921600, bits=8, parity=None, stop=1, timeout_char=1000)

# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
threshold_list = [(0, 255)] # [(200, 255)]

# Resetting the Lepton
#print("Resetting Lepton...")
sensor.reset()
#print("Lepton Res (%dx%d)" % (sensor.ioctl(sensor.IOCTL_LEPTON_GET_WIDTH),
                              #sensor.ioctl(sensor.IOCTL_LEPTON_GET_HEIGHT)))
#print("Radiometry Available: " + ("Yes" if sensor.ioctl(sensor.IOCTL_LEPTON_GET_RADIOMETRY) else "No"))


sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=5000)
clock = time.clock()

while True:
    LED(2).on()
    message = uart.read(9)

    if message is None:
        pass
    elif (b'PIC' in message) or (b'VID' in message):
        min_temp_in_celsius = int(message[3:6].decode("utf-8"))
        max_temp_in_celsius = int(message[6:9].decode("utf-8"))
        sensor.ioctl(sensor.IOCTL_LEPTON_SET_MEASUREMENT_MODE, True)
        sensor.ioctl(sensor.IOCTL_LEPTON_SET_MEASUREMENT_RANGE, min_temp_in_celsius, max_temp_in_celsius)
        #print(min_temp_in_celsius)
        #print(max_temp_in_celsius)

        if b'PIC' in message:
            take_picture()
        else:
            take_video()

