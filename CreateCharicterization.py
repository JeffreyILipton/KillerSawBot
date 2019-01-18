#!/usr/bin/python
from threading import Thread, Lock, Event
from StateEstimator import *
from CreateInterface import *
from KillerInterface import *
from Model import *
from TrajectoryTests import *
import sys

import csv

from plotRun import plotCSVRun


class Charicterizer(Thread):
    def __init__(self,CRC,stateholder,speeds,time_step):
        Thread.__init__(self)
        self.CRC = CRC
        self.holder = stateholder
        self.speeds = speeds
        self.time = time_step
        

        self.CRC.start()
        self.offset = np.matrix([0,0,0]).transpose()
        self.t0 = 0
        self.csvFile = open("Run.csv",'wb')
        self.writer = csv.writer(self.csvFile)
        #row = [s[3],self.Xks[index],X,U]
        row=['Time','U_set','X','Y','Theta']
        self.writer.writerow(row)
        
    def transform(self,X):
        
        th = -self.offset[2,0]
        Rv = np.matrix([[cos(th), -sin(th) ,0],
                       [sin(th), cos(th)  ,0],
                       [0,0,1]])

        return Rv.dot(X-self.offset)

    def run(self):
        first = True
        lastspeed=0
        last_t = 0

        while True:
            
            X = self.holder.GetConfig()
            t = self.holder.getTime()
            
            
            if(first):
                # for first time step set offset and start movement
                self.offset = X
                self.t0 = t
                print 'offset:',  self.offset
                first = False
            
            X=self.transform(X)
            tk = (t-self.t0)/1000000 #convert to seconds since start

            index = int(floor(tk/self.time))

            if ((index > (len(self.speeds)-1) )or tk>(len(self.speeds)*self.time) ):
                self.csvFile.close()
                self.CRC.drive(0,0)
                # self.CRC.stop()
                print "closed"
                break
            else:
                speed = self.speeds[index]
                if(lastspeed != speed): 
                    lastspeed = speed
                    print speed
                    self.CRC.drive(speed,speed)
                    # self.CRC.directDrive(speed,speed)
                if last_t!= t:
                    row = [t,speed]+[X[0,0], X[1,0],  X[2,0] ]
                    self.writer.writerow(row)
                    last_t=t
    
                

def main():
    
    channel = 'VICON_KillerSawBot'#'VICON_create8'
    s = 60
    speeds = [100,0,200,0,300,0]
    time_step = 5 #s




    lock = Lock()
    sh = StateHolder(lock,np.matrix([0,0,0]).transpose())
    VI_stopper = Event()
    VI = ViconInterface(VI_stopper,channel,sh)
    # CRC = CreateRobotCmd('/dev/ttyUSB0',Create_OpMode.Full,Create_DriveMode.Direct)
    CRC = KillerRobotCmd("/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DN04ASN0-if00-port0")
    CC = Charicterizer(CRC,sh,speeds,time_step)


    
    VI.start()
    time.sleep(0.05)
    CC.start()
    #VTL.start()

    CC.join()
    VI_stopper.set()
    VI.join()
    print "Done"
    plotCSVRun()

if __name__ == "__main__":
    sys.exit(int(main() or 0))
