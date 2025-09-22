#!/usr/bin/env python3

import serial
from time import sleep
import struct

# To run this script on Jetson independently, you can do
# eg. python3 serial_handler.py 100 100 100 100
START = 255 # start byte preceding every message
class SerialHandler:
	def __init__(self):
		try:
			self.SER = serial.Serial("/dev/ttyACM0", 115200, timeout = None)
		except serial.SerialException as e:
			print(f"Error: Could not open or close serial port: {e}")

		self.numMotors = 2
		self.bytesPerMotor = 1
		self.totalDataBytes = self.numMotors * self.bytesPerMotor

	def setBytes(self, b):
		self.totalDataBytes = b
		
	def setPort(self, port, baud):
		self.SER.close()
		self.SER = serial.Serial(port, baud, timeout = None)

	# array format: [ir_sensor]
	def send(self, header, data, logger = None): #messageType can be anything
		# print("sending")
		mnum = (1<<8*self.bytesPerMotor)-1 #make sure each send is within maxbyte
		# assert 0 <= header <= mnum
		# self.SER.write(bytes([START]))
		# self.SER.write(header.to_bytes(self.bytesPerMotor, byteorder="big"))
		# logger.warn(f"Wrate {data}")
		self.SER.write(bytes([255]) + bytes(data)) # write the data to serial port
		logger.info(f"Wrote {data} to microcontroller")

	def readMsg(self, logger=None):
		logger.info(f'Serial bytes in waiting: {self.SER.in_waiting}')
		if(self.SER.in_waiting<2): return []
		elif(self.SER.in_waiting>4): self.SER.read((self.SER.in_waiting//2)*2)
		header = self.SER.read(1)
		feedback = list(struct.iter_unpack("B",self.SER.read(1))) # tuple of: ir_sensor
		feedback = [i[0] for i in feedback]
		return feedback


if __name__ == "__main__":
	import sys
	header = 0 # 0 for motor commands
	totalDataBytes = 2
	data = [ord('A') for i in range(totalDataBytes)] # populate a list with A's as default values
	for i in range(1, min(len(sys.argv), 3)): # populate data with (up to 2) args passed into command line
		data[i-1] = int(sys.argv[i])
	print(f"Data {data}")
	handler = SerialHandler()
	while True:
		handler.send(header,data)
		sleep(0.1)