import inputs
import jigsaw_robot

pads = inputs.devices.gamepads
if len(pads) == 0:
    raise IOError("No gamepad found")


def handle(event):
    # left wheel
    if event.code == 'ABS_Y':
        if abs(event.state) > 512 and not jigsaw_robot.relay_module.is_drill_active():
            jigsaw_robot.move_left_wheel(-(event.state/32768)*300)
        else:
            jigsaw_robot.move_left_wheel(0)
    # right wheel
    if event.code == 'ABS_RY':
        if abs(event.state) > 512 and not jigsaw_robot.relay_module.is_drill_active():
            jigsaw_robot.move_right_wheel(-(event.state/32768)*300)
        else:
            jigsaw_robot.move_right_wheel(0)
    # drill activate
    if event.code == 'BTN_TL':
        if event.state:
            jigsaw_robot.activate_drill(True)
        else:
            jigsaw_robot.activate_drill(False)
            jigsaw_robot.move_left_wheel(0)
            jigsaw_robot.move_right_wheel(0)
            jigsaw_robot.raise_drill(5)
    if event.code == 'ABS_Z':
        if event.state > 30 and jigsaw_robot.relay_module.is_drill_active():
            jigsaw_robot.lower_drill((event.state/256)*5)
        else:
            jigsaw_robot.raise_drill(5)
    if event.code == 'BTN_SOUTH':
        if event.state:
            jigsaw_robot.lower_jigsaw(5)
    if event.code == 'BTN_WEST':
        if event.state:
            jigsaw_robot.raise_jigsaw(5)
            jigsaw_robot.activate_jigsaw(False)
    if event.code == 'BTN_TR':
        if event.state: #and jigsaw_robot.slide_module.get_jigsaw_position() > 70:
            jigsaw_robot.activate_jigsaw(True)
        else:
            jigsaw_robot.activate_jigsaw(False)

            

while True:
    events = inputs.get_gamepad()
    for event in events:
        print(event.ev_type, event.code, event.state)
        handle(event)


