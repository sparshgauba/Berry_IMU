#################################################################
# Test program to parse RPi output
#################################################################

import sys
import signal
import time
import math as m
import datetime
import os
import json
import subprocess
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008


def collect(sensor_socket, ADDRESS, signal):
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    melee = 0
    arrow_load = 0
    x = 0
    CLK  = 18
    MISO = 23
    MOSI = 24
    CS   = 25
    mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)
    threshold = 600
    gamma1 = 2.5
    max_limit = 870.0
    value = 0
    force_val = 0.0
    last_val = 0
    last_force = 0.0
    rotary = 0.0
    launch_arrow = False
    launch_force = 0
    time.sleep(3)
    # Starts IMU process
    p = subprocess.Popen("./calib", stdout=subprocess.PIPE)
    # Temporary fix for preventing parsing errors
    # Waits two seconds to let IMU program start
    while x < 10:
        x+=1
        ln = p.stdout.readline()
        time.sleep(0.005)
    while True:
        #Collect force sensor data
        last_val = value
        last_force = force_val
        value = mcp.read_adc(1)
        if value <= threshold:
            force_val = 0.0
        else:
            force_val = ((value - threshold)/(max_limit - threshold))**gamma1
        if force_val > 1:
            force_val = 1
        if last_val > 600 and (last_val - value) > 70:
            launch_arrow = True
            launch_value = last_force
        else:
            launch_arrow = False
            launch_value = force_val
        #Read raw potentiometer value
        rotary = mcp.read_adc(0)
        rotary = (rotary/1023.0)*100
        count = 0
        i1 = 0
        i2 = 0
        i3 = 0
        i4 = 0

        ln = p.stdout.readline()

        for i in ln:
            count += 1
            if i == ',' and i1 == 0 and i2 == 0 and i3 == 0 and i4 == 0:
                roll =  float(ln[0:count-1])
                i1 = count
                continue
            if i == ',' and i1 != 0 and i2 == 0 and i3 == 0 and i4 == 0:
                pitch = float(ln[i1:count-1])
                i2 = count
                continue
            if i == ',' and i1 != 0 and i2 != 0 and i3 == 0 and i4 == 0:
                yaw = float(ln[i2:count-1])
                i3 = count
                continue
            if i == ',' and i1 != 0 and i2 != 0 and i3 != 0 and i4 == 0:
                melee = int(ln[i3:count-1])
                i4 = count
                continue
            if count == len(ln):
                arrow_load = int(ln[i4:count-1])
        package = {"angle1": (roll+90), "angle2": (pitch), "angle3": (-yaw), "force": launch_value, "launch": launch_arrow, "reload": arrow_load, "melee": melee}
        package_string = json.dumps(package)
        sensor_socket.sendto(package_string, ADDRESS)
