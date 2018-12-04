from vicon import body_t
from math import *
from threading import Thread, Lock, Event
import time
import numpy as np
import os
if os.name!='nt':
    import lcm


import sys

import csv

class StateHolder():
    def __init__(self,lock,start,t=0):
        self.state = start
        self.t=t
        self.lock = lock
    def setState(self,state,t):
        self.lock.acquire()
        self.state = state
        self.t=t
        self.lock.release()
	#print "set: ",state
    def GetConfig(self):
        return self.state
    def getTime(self):
        return self.t


def vicon_handler(holder,channel,data):
    msg = body_t.decode(data)
    t = msg.utime
    position = msg.trans
    s,x,y,z = msg.quat
    a = 2*atan2(z,s)
    if a<0: a = 2*pi+a
    state = np.matrix([position[0]*1000.0,position[1]*1000.0,a]).transpose() #[mm,mm,rad],ms since epoch]
    #print 'handeled:', state
    holder.setState(state,t) # [mm,mm,rad], ms since epoch]

class ViconInterface(Thread):
    def __init__(self,stopper,channel,state_holder):
        self.stopper = stopper
        self.channel = channel
        Thread.__init__(self)
        self.lc = lcm.LCM()
        handler = lambda x,y: vicon_handler(state_holder,x,y)
        self.subscription = self.lc.subscribe(channel,handler)

    def run(self):
        while not self.stopper.is_set():
            self.lc.handle()


class ViconTester(Thread):
    def __init__(self,state_holder,rate):
        Thread.__init__(self)
        self.holder = state_holder
        self.rate=rate
	    #print "interval: ",1.0/self.rate
    def run(self):
        while True:
            time.sleep(1.0/self.rate)
            print "rate:%0.2f"%self.rate, self.holder.GetConfig()

class ViconLogger(Thread):
    def __init__(self,name,state_holder,rate):
        Thread.__init__(self)
        self.holder = state_holder
        self.rate=rate
        self.csvFile = open(name,'wb')
        self.writer = csv.writer(self.csvFile)

	    #print "interval: ",1.0/self.rate
    def run(self):
        n=1
        while n<10:
            print n
            n+=1
            time.sleep(1.0/self.rate)
            v = self.holder.getState()
            self.writer.writerow(v)
        self.csvFile.close()

def main():
    #http://www.tutorialspoint.com/python/python_multithreading.htm
    #http://www.toptal.com/python/beginners-guide-to-concurrency-and-parallelism-in-python
    #http://lcm.googlecode.com/svn/www/reference/lcm/tut_python.html
    channel = 'VICON_create8'
    lock = Lock()

    sh = StateHolder(lock,[0,0,0])
    stopper = Event()

    VI = ViconInterface(stopperchannel,sh)
    #VT1 = ViconTester(sh,10)
    VT2 = ViconTester(sh,1)
    VI.start()
    #VT1.start()
    VT2.start()

    
    #VT1.join()
    VT2.join()
    stopper.set()
    VI.join()
    print "Exiting Main Thread"


    if __name__ == "__main__":
        sys.exit(int(main() or 0))
