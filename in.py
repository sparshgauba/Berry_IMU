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

p = 0

def handler(signum, frame):
    print ("Program ended")
    p.kill()

def main():
    # Starts IMU process
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
        count = 0
        i1 = 0
        i2 = 0

        ln = p.stdout.readline()

        for i in ln:
            count += 1
            if i == ',' and i1 == 0 and i2 == 0:
                roll =  float(ln[0:count-1])
                roll = roll * 180/m.pi
                i1 = count
                continue
            if i == ',' and i1 != 0 and i2 == 0:
                pitch = float(ln[i1:count-1])
                pitch = pitch * 180/m.pi
                i2 = count
                continue
            if count == len(ln):
                yaw = float(ln[i2:count-1])
                yaw = yaw * 180/m.pi


        print (roll,pitch,yaw)
        time.sleep(0.020)



        
signal.signal(signal.SIGINT, handler)
if __name__ == "__main__":
    main()
