#!usr/bin/python3
#this auto inits the robot
import serial
import pickle
import time
import sys
from enum import IntEnum

import jigsaw_robot

UNLIKELY_NEWLINE = "THIS IS NOT GOOD CODING PRACTICE".encode('utf-8')

class KillerRobotProtocol(IntEnum):
    Start = 0
    LeftMotor = 1
    RightMotor = 2
    LeftAndRightMotor = 3
    Drill = 4
    JigsawLower = 5
    JigsawRaise = 6
    JigsawStart = 7
    JigsawStop = 8
    Status = 9
    Stop = 10

class KillerRobotOutMessage(object):
    def __init__(self, message_type , first_value = None, second_value = None):
        self.message_type = message_type
        self.first_value = first_value
        self.second_value = second_value

    def serialized(self):
        return pickle.dumps(self)

    def __str__(self):
        return str(self.message_type) +","+ str(self.first_value) +","+ str(self.second_value)

class KillerRobotInMessage(object):
    def __init__(self, ack , error_type  = None):
        self.ack = ack
        self.error_type = error_type

    def serialized(self):
        return pickle.dumps(self)

    def __str__(self):
        return "InMsg" + "," + str(self.ack) + "," + str(self.error_type)

class KillerRobotClient(object):
	def __init__(self, port = None):
		self.port = serial.Serial(port)
		self.active = False

	def start(self):
		self.active = True
		self.port.reset_input_buffer()
		self.port.reset_output_buffer()
		while(self.active):
			self._poll_for_commands()

		self.stop()

	def stop(self):
		self.port.close()
		jigsaw_robot.panic()

	def _poll_for_commands(self):
		in_message = self.port.read_until(UNLIKELY_NEWLINE)
		in_message = in_message[0: len(in_message) - len(UNLIKELY_NEWLINE)]
		in_command = pickle.loads(in_message)
		response = None
		if isinstance(in_command, KillerRobotOutMessage):
			self._execute_command(in_command)
			response = KillerRobotInMessage(True)
		else:
			response = KillerRobotInMessage(False, "Wrong input type")
		self.port.write(response.serialized())
		self.port.write(UNLIKELY_NEWLINE)
		print(response.serialized(), UNLIKELY_NEWLINE)

	def _execute_command(self, in_command : KillerRobotOutMessage):
		command = in_command.message_type
		if command == KillerRobotProtocol.Start:
			print("Just got a command")
			# don't know what to do here lol
		if command == KillerRobotProtocol.Stop:
			self.active = False
		if command == KillerRobotProtocol.LeftMotor:
			jigsaw_robot.move_left_wheel(in_command.first_value)
		if command == KillerRobotProtocol.RightMotor:
			jigsaw_robot.move_right_wheel(in_command.first_value)
		if command == KillerRobotProtocol.LeftAndRightMotor:
			jigsaw_robot.move_left_wheel(in_command.first_value)
			jigsaw_robot.move_right_wheel(in_command.second_value)
		if command == KillerRobotProtocol.Drill:
			jigsaw_robot.activate_drill(True)
			jigsaw_robot.lower_drill(1)
			time.sleep(30)
			jigsaw_robot.raise_drill(5)
			jigsaw_robot.activate_drill(False)
			time.sleep(10)
		if command == KillerRobotProtocol.JigsawLower:
			jigsaw_robot.lower_jigsaw(5)
		if command == KillerRobotProtocol.JigsawRaise:
			jigsaw_robot.raise_jigsaw(5)
		if command == KillerRobotProtocol.JigsawLower:
			jigsaw_robot.lpower_jigsaw(5)
		if command == KillerRobotProtocol.JigsawStart:
			jigsaw_robot.activate_jigsaw(True)
		if command == KillerRobotProtocol.JigsawStop:
			jigsaw_robot.activate_jigsaw(False)
			
def main():
	client = KillerRobotClient("/dev/ttyUSB0")
	client.start()

if __name__ == "__main__":
    sys.exit(int(main() or 0))
