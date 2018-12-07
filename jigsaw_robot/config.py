from math import pi

DRIVE_MODULE_PORT = "/dev/serial/by-id/usb-03eb_USB_Roboclaw_2x30A-if00"
DRIVE_MODULE_ADDR = 0x80
DRIVE_TICKS_PER_LINEAR_MM = 4776.384 / (73*pi)

SLIDE_MODULE_PORT = "/dev/serial/by-id/usb-03eb_USB_Roboclaw_2x15A-if00"
SLIDE_MODULE_ADDR = 0x80
SLIDE_TICKS_PER_LINEAR_MM = 739.908 / 2

DRILL_PLUNGE_DEPTH = 50
JIGSAW_PLUNGE_DEPTH = 75

DRILL_ACTIVATE_PIN = 8
JIGSAW_ENABLE_PIN = 7
JIGSAW_ACTIVATE_PIN = 25