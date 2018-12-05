from jigsaw_robot.config import (
    SLIDE_MODULE_PORT,
    SLIDE_MODULE_ADDR,
    SLIDE_TICKS_PER_LINEAR_MM,
    DRILL_PLUNGE_DEPTH,
    JIGSAW_PLUNGE_DEPTH
    )

from jigsaw_robot.roboclaw.roboclaw import Roboclaw

slide = Roboclaw(SLIDE_MODULE_PORT, 115200)
slide.Open()

# home_axis()

"""
Gets drill motor encoder value converted to mm
"""
def get_drill_position():
    valid, position, _ = slide.ReadEncM1(SLIDE_MODULE_ADDR)
    if not valid:
        raise IOError("Failed to read drill motor encoder")
    return position/SLIDE_TICKS_PER_LINEAR_MM

"""
Gets jigsaw motor encoder value converted to mm
"""
def get_jigsaw_position():
    valid, position, _ = slide.ReadEncM2(SLIDE_MODULE_ADDR)
    if not valid:
        raise IOError("Failed to read jigsaw motor encoder")
    return position/SLIDE_TICKS_PER_LINEAR_MM

"""
Moves drill to top position
"""
def raise_drill(speed):
    speed = int(speed * SLIDE_TICKS_PER_LINEAR_MM)
    accel = int(5*speed)
    deccel = accel
    position = int(1 * SLIDE_TICKS_PER_LINEAR_MM)
    if not slide.SpeedAccelDeccelPositionM1(SLIDE_MODULE_ADDR,accel,speed,deccel,position,1):
        raise IOError("Failed to raise drill: CRC failure")

"""
Moves drill to bottom position
speed - The speed at which to move.
"""
def lower_drill(speed):
    speed = int(speed * SLIDE_TICKS_PER_LINEAR_MM)
    accel = int(5*speed)
    deccel = accel
    position = int(DRILL_PLUNGE_DEPTH * SLIDE_TICKS_PER_LINEAR_MM)
    if not slide.SpeedAccelDeccelPositionM1(SLIDE_MODULE_ADDR,accel,speed,deccel,position,1):
        raise IOError("Failed to lower drill: CRC failure")

"""
Moves jigsaw to top position
"""
def raise_jigsaw(speed):
    speed = int(speed * SLIDE_TICKS_PER_LINEAR_MM)
    accel = int(5*speed)
    deccel = accel
    position = int(1 * SLIDE_TICKS_PER_LINEAR_MM)
    if not slide.SpeedAccelDeccelPositionM2(SLIDE_MODULE_ADDR,accel,speed,deccel,position,1):
        raise IOError("Failed to raise jigsaw: CRC failure")

"""
Moves jigsaw to bottom position
"""
def lower_jigsaw(speed):
    speed = int(speed * SLIDE_TICKS_PER_LINEAR_MM)
    accel = int(5*speed)
    deccel = accel
    position = int(JIGSAW_PLUNGE_DEPTH * SLIDE_TICKS_PER_LINEAR_MM)
    if not slide.SpeedAccelDeccelPositionM2(SLIDE_MODULE_ADDR,accel,speed,deccel,position,1):
        raise IOError("Failed to lower jigsaw: CRC failure")

"""
Stops all motors
"""
def panic():
    success = slide.SpeedM1(SLIDE_MODULE_ADDR, 0)
    success = success and slide.SpeedM2(SLIDE_MODULE_ADDR, 0)
    if not success:
        raise IOError("Slide panic failed. Time to really panic!")