import sys
import signal
import time
import math
import datetime
import os
import json
import subprocess
# Import SPI library (for hardware SPI) and MCP3008 library.                                                                                                  
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

def collect(sensor_socket, ADDRESS, signal):

    #Force Sensor Values
    CLK  = 18
    MISO = 23
    MOSI = 24
    CS   = 25
    mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)
    threshold = 500
    gamma1 = 2.5
    gamma2 = 0.65
    max_limit = 1023.0
    gain = 0.4
    force_val = 0.0
    last_val = 0.0

    #IMU Values
    p = subprocess.Popen("./calib", stdout=subprocess.PIPE)
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    x = 0

    # Temporary fix for preventing parsing errors
    # Waits two seconds to let IMU program start

    while x < 100:
        x+=1
        ln = p.stdout.readline()
        time.sleep(0.020)

    while True:
        #Collect force sensor data
        #value = mcp.read_adc(1)
        #if value <= threshold:
            #force_val = 0.0
        #else:
            #last_val = force_val
            #if value <= 600:
                #force_val = ((value - threshold)/(max_limit - threshold))**gamma1
            #else:
                #force_val = ((value - threshold)/(max_limit - threshold))**gamma1 + gain*((value - threshold - 50)/(max_limit - threshold))**gamma2



        #Parse IMU value
        count = 0
        i1 = 0
        i2 = 0

        ln = p.stdout.readline()

        for i in ln:
            count += 1
            if i == ',' and i1 == 0 and i2 == 0:
                roll =  float(ln[0:count-1])
                i1 = count
                continue
            if i == ',' and i1 != 0 and i2 == 0:
                pitch = float(ln[i1:count-1])
                i2 = count
                continue
            if count == len(ln):
                yaw = float(ln[i2:count-1])

        package = {"angle1": (roll), "angle2": (pitch), "angle3": (yaw), "force": force_val}
        package_string = json.dumps(package)
        sensor_socket.sendto(package_string, ADDRESS)
        time.sleep(0.02)
