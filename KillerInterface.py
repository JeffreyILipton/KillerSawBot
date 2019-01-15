import serial
import pickle
import sys
import time
from Logging import Queue
from enum import IntEnum

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
        return pickle.dumps(self, protocol = 2)

    def __str__(self):
        return str(self.message_type) +","+ str(self.first_value) +","+ str(self.second_value)

class KillerRobotInMessage(object):
    def __init__(self, ack , error_type  = None):
        self.ack = ack
        self.error_type = error_type

    def serialized(self):
        return pickle.dumps(self, protocol = 2)

    def __str__(self):
        return "InMsg" + "," + str(self.ack) + "," + str(self.error_type)

class KillerRobotCmd(object):
    """
    Initializes Killer robot.
    port - The port that the Xbee is connected to. Used for serial comms
    debug - If set to a queue instance, this robot instance will redirect all serial commands to the queue file
    """
    def __init__(self, port , log_queue = None):
        # robot velocity limits for both wheels
        self.velocity_min = -100 #mm/s
        self.velocity_max = 100 #mm/s
        self.is_logging = True if log_queue else False
        self.logQueue = log_queue

        self.is_debug = False if port else True
        self.port = None
        if not self.is_debug:
            self.port = serial.Serial(port)

    """
    Starts up robot; opens port and tells robot to wake up.
    """
    def start(self):
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()
        # if self.port: self.port.open()
        start_message = KillerRobotOutMessage(KillerRobotProtocol.Start)
        self._write_message(start_message)

    """
    Stops robot; closes port and tells robot to shut down.
    """
    def stop(self):
        stop_message = KillerRobotOutMessage(KillerRobotProtocol.Stop)
        self._write_message(stop_message)
        if self.port: self.port.close()

    """
    Tells robot to drive with wheels moving at requested velocity.
    Command will FAIL if the robot is drilling or moving the jigsaw slide
    """
    def drive(self, left_velocity, right_velocity):
        left_velocity = max(self.velocity_min, min(self.velocity_max, left_velocity))
        right_velocity = max(self.velocity_min, min(self.velocity_max, right_velocity))
        drive_message = KillerRobotOutMessage(KillerRobotProtocol.LeftAndRightMotor,
                                           first_value = left_velocity,
                                           second_value = right_velocity)
        self._write_message(drive_message)

    """
    Tells robot to peck drill at the current location. Command will FAIL
    if the robot is moving.
    """
    def drill(self):
        drill_message = KillerRobotOutMessage(KillerRobotProtocol.Drill)
        self._write_message(drill_message)


    """
    Tells the robot to lower the jigsaw
    """
    def lower_jigsaw(self):
        jigsaw_message = KillerRobotOutMessage(KillerRobotProtocol.JigsawLower)
        self._write_message(jigsaw_message)

    """
    Lowers jigsaw. Command will FAIL if the robot is moving
    """
    def raise_jigsaw(self):
        jigsaw_message = KillerRobotOutMessage(KillerRobotProtocol.JigsawRaise)
        self._write_message(jigsaw_message)
    """
    Starts jigsaw.
    """
    def start_jigsaw(self):
        jigsaw_message = KillerRobotOutMessage(KillerRobotProtocol.JigsawStart)
        self._write_message(jigsaw_message)
    """
    Stops jigsaw
    """
    def stop_jigsaw(self):
        jigsaw_message = KillerRobotOutMessage(KillerRobotProtocol.JigsawStop)
        self._write_message(jigsaw_message)


    def _write_message(self, message):
        if self.is_logging:
            self.logQueue.put(str(message).split(','))

        response = None    
        if self.is_debug:
            response = KillerRobotInMessage(True)
        else:
            self.port.write(message.serialized())
            self.port.write(UNLIKELY_NEWLINE)
            response = self.port.read_until(UNLIKELY_NEWLINE)
            response = response[0: len(response) - len(UNLIKELY_NEWLINE)]
            response = pickle.loads(response)
        if self.is_logging:
            self.logQueue.put(str(response).split(','))
        
        if not response.ack:
            raise IOError("Robot did not successfully parse command")
        return response

def main():
    cmd = KillerRobotCmd("/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DN04ASN0-if00-port0")
    cmd.start()
    cmd.drive(10, 10)
    time.sleep(3)
    cmd.drive(0, 0)
    cmd.stop()

    return 0

if __name__ == "__main__":
    sys.exit(int(main() or 0))