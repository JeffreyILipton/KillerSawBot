#!/usr/bin/python
import serial
import time
from enum import IntEnum


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


class KillerRobotCmd(object):
    """
    Initializes Killer robot.
    port - The port that the Xbee is connected to. Used for serial comms
    debug - If set to a queue instance, this robot instance will redirect all serial commands to the queue file
    """

    def __init__(self, port, log_queue=None):
        # robot velocity limits for both wheels
        self.velocity_min = -500  # mm/s
        self.velocity_max = 500  # mm/s
        self.is_logging = True if log_queue else False
        self.logQueue = log_queue

        self.is_debug = False if port else True
        self.port = None
        if not self.is_debug:
            self.port = serial.Serial(port, timeout=10)

    """
    Starts up robot; opens port and tells robot to wake up.
    """

    def start(self):
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()
        self._write_message(KillerRobotProtocol.Start)

    """
    Stops robot; closes port and tells robot to shut down.
    """

    def stop(self):
        self._write_message(KillerRobotProtocol.Stop)
        if self.port:
            self.port.close()

    """
    Tells robot to drive with wheels moving at requested velocity.
    Command will FAIL if the robot is drilling or moving the jigsaw slide
    """

    def drive(self, left_velocity, right_velocity):
        left_velocity = max(self.velocity_min, min(
            self.velocity_max, left_velocity))
        right_velocity = max(self.velocity_min, min(
            self.velocity_max, right_velocity))
        self._write_message(
            KillerRobotProtocol.LeftAndRightMotor, left_velocity, right_velocity)

    """
    Tells robot to peck drill at the current location.
    if the robot is moving.
    """

    def drill(self):
        self._write_message(KillerRobotProtocol.Drill)

    """
    Tells the robot to lower the jigsaw
    """

    def lower_jigsaw(self):
        self._write_message(KillerRobotProtocol.JigsawLower)

    """
    Lowers jigsaw. Command will FAIL if the robot is moving
    """

    def raise_jigsaw(self):
        self._write_message(KillerRobotProtocol.JigsawRaise)
    """
    Starts jigsaw.
    """

    def start_jigsaw(self):
        self._write_message(KillerRobotProtocol.JigsawStart)
    """
    Stops jigsaw
    """

    def stop_jigsaw(self):
        self._write_message(KillerRobotProtocol.JigsawStop)

    def _write_message(self, action, value1=None, value2=None):
        message = str(int(action))
        if value1:
            message = message + str(value1) + ","
            if value2:
                message = message + str(value2)

        if self.is_logging:
            self.logQueue.put(message)
        else:
            self.port.write(message.encode('utf-8') + '\n')

def main():
    cmd = KillerRobotCmd("/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DN04ASN0-if00-port0")
    cmd.start()
    cmd.drive(500, 500)
    time.sleep(1)
    cmd.drive(-500, -500)
    time.sleep(1)
    cmd.drive(500, -500)
    time.sleep(1)
    cmd.drive(-500, 500)
    time.sleep(1)
    cmd.drive(0, 0)
    cmd.stop()

    return 0


if __name__ == "__main__":
    sys.exit(int(main() or 0))