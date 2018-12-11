#!usr/bin/python3
#this auto inits the robot
import serial
import pickle
import time

import jigsaw_robot
from KillerInterface import (
	KillerRobotProtocol,
	KillerRobotInMessage,
	KillerRobotOutMessage,
	UNLIKELY_NEWLINE)

class KillerRobotClient(object):
	def __init__(self, port = None):
		self.port = serial.Serial(port)
		self.active = False

	def start(self):
		self.active = True
		while(active):
			poll_for_commands()

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
			_execute_command(in_command)
			response = KillerRobotInMessage(True)
		else:
			response = KillerRobotInMessage(False, "Wrong input type")
		serial.write(response.serialized())
		serial.write('\n'.encode())

	def _execute_command(in_command : KillerRobotOutMessage):
		command = in_command.message_type
		if command == KillerRobotProtocol.Start:
			pass
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
	client = KillerRobotClient("/dev/serial")
	client.start()

if __name__ == "__main__":
    sys.exit(int(main() or 0))
