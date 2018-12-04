﻿from threading import Thread, Lock
from CreateStateEstimator import *
from CreateInterface import *
from CreateModel import *
from TrajectoryTests import *
import sys

import csv

from plotRun import plotCSVRun


class CreateCharicterizer(Thread):
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
                self.CRC.stop()
                print "closed"
                break
            else:
                speed = self.speeds[index]
                if(lastspeed != speed): 
                    lastspeed = speed
                    print speed
                    self.CRC.directDrive(speed,speed)
                if last_t!= t:
                    row = [t,speed]+[X[0,0], X[1,0],  X[2,0] ]
                    self.writer.writerow(row)
                    last_t=t
    
                

def main():
    
    channel = 'VICON_sawbot'#'VICON_create8'
    s = 60
    speeds = [10,0,20,0,30,0]
    time_step = 5 #s




    lock = Lock()
    sh = StateHolder(lock,np.matrix([0,0,0]).transpose())

    VI = ViconInterface(channel,sh)
    CRC = CreateRobotCmd('/dev/ttyUSB0',Create_OpMode.Full,Create_DriveMode.Direct)
    CC = CreateCharicterizer(CRC,sh,speeds,time_step)


    
    VI.start()
    time.sleep(0.05)
    CC.start()
    #VTL.start()

    CC.join()
    #VI.join()
#    VTL.join()
    print "Done"
    plotCSVRun()

if __name__ == "__main__":
    sys.exit(int(main() or 0))
