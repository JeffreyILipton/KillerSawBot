import time
import pigpio

from jigsaw_robot.config import (
    DRILL_ACTIVATE_PIN,
    JIGSAW_ACTIVATE_PIN,
    JIGSAW_ENABLE_PIN,
)

pi = pigpio.pi()

if not pi.connected:
    raise IOError("Could not connect to PI. Maybe the daemon wasn't started?")

pi.set_mode(DRILL_ACTIVATE_PIN, pigpio.OUTPUT)
pi.set_mode(JIGSAW_ENABLE_PIN, pigpio.OUTPUT)
pi.set_mode(JIGSAW_ACTIVATE_PIN, pigpio.OUTPUT)

pi.write(DRILL_ACTIVATE_PIN, 1)
pi.write(JIGSAW_ENABLE_PIN, 1)
pi.write(JIGSAW_ACTIVATE_PIN, 1)

"""
Turns on/off the drill.
is_active - Whether the drill should be ACTIVATE or turned off.
"""


def activate_drill(is_active):
    pi.write(DRILL_ACTIVATE_PIN, 0 if is_active else 1)


def is_drill_active():
    return False if pi.read(DRILL_ACTIVATE_PIN) else True


"""
Turns on/off the jigsaw.
is_active - Whether the jigsaw should be ACTIVATE or turned off.
"""


def activate_jigsaw(is_active):
    pi.write(JIGSAW_ACTIVATE_PIN, 0 if is_active else 1)
    pi.write(JIGSAW_ENABLE_PIN, 0 if is_active else 1)


def is_jigsaw_active():
    return False if pi.read(JIGSAW_ACTIVATE_PIN) else True


"""
Stops all tools from running.
"""


def panic():
    pi.write(DRILL_ACTIVATE_PIN, 1)
    pi.write(JIGSAW_ENABLE_PIN, 1)
    pi.write(JIGSAW_ACTIVATE_PIN, 1)
