#!/usr/bin/python
#################################################################
# Test program to parse orientation output
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

def handler(signum, frame):
    print ("Program ended")
    p.kill()

def main():
    signal.signal(signal.SIGINT, handler)
    # Starts IMU process
    p = subprocess.Popen("./calib", stdout=subprocess.PIPE)
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
    gamma2 = 0.65
    max_limit = 930.0
    gain = 0.4
    value = 0
    force_val = 0.0
    last_val = 0
    last_force = 0.0
    rotary = 0.0
    launch_arrow = False
    launch_force = 0
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
            if value <= 800:
                force_val = ((value - threshold)/(max_limit - threshold))**gamma1
            else:
                force_val = ((value - threshold)/(max_limit - threshold))**gamma1 + gain*((value - threshold - 50)/(max_limit - threshold))**gamma2
        force_val = force_val * 100
        if (last_val - value) > 50:
            launch_arrow = True
            launch_value = last_force
        else:
            launch_arrow = False
            launch_value = 0
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


        #print ("%.2f               %.2f                %.2f" %roll,pitch,yaw)
        print(roll,pitch,yaw,melee,arrow_load,launch_arrow,launch_value)
        time.sleep(0.01)

if __name__ == "__main__":
    main()
