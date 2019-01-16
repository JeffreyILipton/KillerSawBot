#!/usr/bin/python3
import serial
import time
import sys
from enum import IntEnum

import jigsaw_robot


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


class KillerRobotClient(object):
    def __init__(self, port=None):
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
        jigsaw_robot.panic()

    def _poll_for_commands(self):
        input_string = self.port.readline().decode('ascii')[0:-1]
        input_values = input_string.split(",")
        print("Command received!", input_string)
        self._execute_command(input_values)

    def _execute_command(self, input_values):
        command = input_values[0]
        if command == KillerRobotProtocol.Start:
            print("Initialized")
            # don't know what to do here lol
        if command == KillerRobotProtocol.Stop:
            self.stop()
        if command == KillerRobotProtocol.LeftMotor:
            jigsaw_robot.move_left_wheel(float(input_values[1]))
        if command == KillerRobotProtocol.RightMotor:
            jigsaw_robot.move_right_wheel(float(input_values[1]))
        if command == KillerRobotProtocol.LeftAndRightMotor:
            jigsaw_robot.move_left_wheel(float(input_values[1]))
            jigsaw_robot.move_right_wheel(float(input_values[2]))
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
            jigsaw_robot.lower_jigsaw(5)
        if command == KillerRobotProtocol.JigsawStart:
            jigsaw_robot.activate_jigsaw(True)
        if command == KillerRobotProtocol.JigsawStop:
            jigsaw_robot.activate_jigsaw(False)


def main():
    client = KillerRobotClient("/dev/ttyUSB0")
    client.start()


if __name__ == "__main__":
    sys.exit(int(main() or 0))