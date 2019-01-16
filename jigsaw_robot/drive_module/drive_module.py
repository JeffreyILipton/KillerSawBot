from jigsaw_robot.config import (
    DRIVE_MODULE_PORT,
    DRIVE_MODULE_ADDR,
    DRIVE_TICKS_PER_LINEAR_MM
)

from jigsaw_robot.roboclaw.roboclaw import Roboclaw

drive = Roboclaw(DRIVE_MODULE_PORT, 115200)

opened = drive.Open()
if not opened:
    raise IOError(
        "Failed to connect to Roboclaw on port {}".format(DRIVE_MODULE_PORT))


"""
Set left motor to go at a certain speed.
"""


def set_left_velocity(velocity: float):
    drive.SpeedM1(DRIVE_MODULE_ADDR, int(velocity * DRIVE_TICKS_PER_LINEAR_MM))


"""
Set right motor to go at a certain speed.
"""


def set_right_velocity(velocity: float):
    drive.SpeedM2(DRIVE_MODULE_ADDR, int(velocity * DRIVE_TICKS_PER_LINEAR_MM))


"""
Gets left motor encoder value converted to mm
"""


def get_left_position():
    valid, position, _ = drive.ReadEncM1(DRIVE_MODULE_ADDR)
    if not valid:
        raise IOError("Failed to read left motor encoder")
    return position / DRIVE_TICKS_PER_LINEAR_MM


"""
Gets right motor encoder value converted to mm
"""


def get_right_position():
    valid, position, _ = drive.ReadEncM2(DRIVE_MODULE_ADDR)
    if not valid:
        raise IOError("Failed to read right motor encoder")
    return position / DRIVE_TICKS_PER_LINEAR_MM


"""
Resets left and right encoder positions.
"""


def reset_encoders():
    drive.ResetEncoders(DRIVE_MODULE_ADDR)


"""
Shuts down both motors.
"""


def panic():
    set_left_velocity(0)
    set_right_velocity(0)


reset_encoders()
