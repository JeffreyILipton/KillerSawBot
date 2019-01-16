from jigsaw_robot.drive_module import drive_module
from jigsaw_robot.slide_module import slide_module
from jigsaw_robot.relay_module import relay_module


"""
Raises the drill to the home position at the max possible speed.
Throws if this cannot be done.
"""


def raise_drill(speed: float):
    slide_module.raise_drill(speed)


"""
Lowers the drill into the workpiece at the entered speed.
Depth is controlled in config.py
speed - Speed to move down at, given in units of mm/s.
Returns nothing immediately.
"""


def lower_drill(speed: float):
    slide_module.lower_drill(speed)


"""
Turns on/off the drill.
is_active - Whether the drill should be active or turned off.
Throws error if this is unsafe.
"""


def activate_drill(is_active: bool):
    relay_module.activate_drill(is_active)


"""
Raises the jigsaw to the home position at the max possible speed.
Throws if this cannot be done.
"""


def raise_jigsaw(speed: float):
    slide_module.raise_jigsaw(speed)


"""
Lowers the jigsaw into the workpiece at the entered speed.
Depth is controlled in config.py
speed - Speed to move down at, given in units of mm/s.
"""


def lower_jigsaw(speed: float):
    slide_module.lower_jigsaw(speed)


"""
Turns on/off the jigsaw.
is_active - Whether the jigsaw should be active or turned off.
Throws error if this is unsafe.
"""


def activate_jigsaw(is_active: bool):
    relay_module.activate_jigsaw(is_active)


"""
Moves left motor at specified speed.
speed - Speed to move at, given in units of mm/s.
"""


def move_left_wheel(speed: float):
    drive_module.set_left_velocity(speed)


"""
Moves right motor at specified speed.
speed - Speed to move at, given in units of mm/s.
"""


def move_right_wheel(speed: float):
    drive_module.set_right_velocity(speed)


"""
Stops all movement.
"""


def panic():
    drive_module.panic()
    relay_module.panic()
    slide_module.panic()
