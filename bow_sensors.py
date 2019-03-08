#!/usr/bin/python
import socket
import threading
import time
import BerryIMU
import fcntl
import struct

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

def socket_create():
	##Creating UDP socket to send sensor data to server
	global sensor_socket
	sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	##Binding sensor data socket to IP address
	HOST = get_ip_address('wlan0')
	SENSOR_PORT = 10000
	SENSOR_ADDRESS = (HOST, SENSOR_PORT)
	sensor_socket.bind(SENSOR_ADDRESS)
	
def data_collect():
	##Sending signals through Pi socket
	PORT = 10000
	SERVER_ADDRESS = "131.179.27.201"
	ADDRESS = (SERVER_ADDRESS, PORT)
	##Start sequence to establish connection
	for _ in range(3):
		sensor_socket.sendto("connect".encode(), ADDRESS)
	##Send sensor data until stop/start signal received
	##Keep listening until program closed
	BerryIMU.collect(sensor_socket, ADDRESS, signal)
		
def get_signal():
	##Grabs cue from Unity to start or stop taking sensor data
	while True:
		data,addr = sensor_socket.recvfrom(4096)
		print (data.decode())
		if data == "collect":
			signal.set()
		elif data == "stop":
			signal.clear()
		
def main():
	socket_create()
	##Generate signal
	global signal
	signal = threading.Event()
	signal.clear()
	##Define threads
	sendDataThread = threading.Thread(target = data_collect)
	getSignalThread = threading.Thread(target = get_signal)
	##Kill threads when main thread dies
	sendDataThread.daemon = True
	getSignalThread.daemon = True
	##Start threads
	sendDataThread.start()
	getSignalThread.start()
	while True:
		time.sleep(0.01)

if __name__ == "__main__":
	main()
