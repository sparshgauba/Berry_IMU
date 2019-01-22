import sys
import signal
import time
import math
import datetime
import os
import json
import subprocess

p = 0

def handler(signum, frame):
    print ("Program ended")
    p.kill()

def main():
    p = subprocess.Popen("./calib", stdout=subprocess.PIPE)
    while True:
            ln = p.stdout.readline()
            print ln
            time.sleep(0.01)

signal.signal(signal.SIGINT, handler)
if __name__ == "__main__":
    main()
