
from CreateInterface import *

def main():
    CRC = CreateRobotCmd('/dev/ttyUSB0',Create_OpMode.Full,Create_DriveMode.Direct)
    print CRC.port.isOpen()
    if CRC.port.isOpen() or DEBUG:
        print "stopping"
        CRC.start()
        CRC.stop()
        print "Done"
        return 0

if __name__ == "__main__":
    sys.exit(int(main() or 0))