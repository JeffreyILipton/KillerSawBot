from threading import Thread, Lock, Event
from StateEstimator import *
from CreateInterface import *
from Model import *
from Controller import *
from TrajectoryTests import *
from Logging import *
import sys
from KillerInterface import *
from plotRun import *

class TickTock():
    def __init__(self):
        self.SimI=0
        self.ConI=0
        self.lock = Lock()
    def setSimI(self,I):
        #self.lock.acquire()
        self.SimI = I
        #self.lock.release()
    def setConI(self,I):
        #self.lock.acquire()
        self.ConI = I
        #self.lock.release()
	#print "set: ",state
    def simTick(self):
        return self.ConI>=self.SimI
    def conTick(self):
        return self.ConI<=self.SimI

class Simulator(Thread):
    def __init__(self,stopper, CRC_sim, stateholder, XKs, ro, dt, Q, speedup = 10, timelock=None, NoNoise=False):
        Thread.__init__(self)
        self.stopper = stopper
        self.CRC = CRC_sim
        self.holder = stateholder
        self.ro =ro
        self.dt = dt
        self.Q = Q
        self.Xks = XKs
        self.t0 = int(time.time()*1000)
        self.index = 0
        self.speedup=speedup
        self.ticktock = timelock
        self.no_noise = NoNoise
    
    def run(self):
        X_k = np.mat(np.zeros( (6,1) ) )
        print "start sim"
        while (not self.stopper.is_set() ) and self.index<( len(self.Xks) ):
            time.sleep(self.dt/self.speedup)
            Uc = self.CRC.LastU()

            G,V = MotorGainAndOffset()
            U = G.dot(Uc)+V

            
            for i in range(0,2):
                if fabs(U[i])<10:
                    U[i]=0

            

            X_k  = self.holder.GetConfig()

            #if (self.index==0): Conf[0,0]+=20.0
            n = [0.1,0.1,2*pi/360/4]
            W = np.matrix([np.random.normal(0,n[i],1) for i in range(0,3)] )

            #print W

            K1 = B(X_k[2,0],self.ro).dot(U)
            K2 = B(X_k[2,0]+self.dt*0.5*K1[2,0],self.ro).dot(U)
            K3 = B(X_k[2,0]+self.dt*0.5*K2[2,0],self.ro).dot(U)
            K4 = B(X_k[2,0]+self.dt*K3[2,0],self.ro).dot(U)



            #    B returns a 3x1 but X_k is 3x1
            X_k_p1 = A(self.dt).dot(X_k) + self.dt/6*(K1+2*K2+2*K3+K4) +W
            if self.no_noise:
                X_k_p1 = A(self.dt).dot(X_k) + self.dt/6*(K1+2*K2+2*K3+K4)
            #print A(self.dt)
            #print X_k_p1[0:3]
            X_k_p1[2,0] = X_k_p1[2,0]%(2.0*pi)
            X_k = X_k_p1
            #print "Sim:",self.index
            
            t = self.t0+self.dt*1000*self.index

            step = False
            if self.ticktock == None: step=True
            elif self.ticktock.simTick(): 
                step = True
                self.ticktock.setSimI(self.index+1)

            if step:
                self.holder.setState(X_k_p1[0:3],t)
                self.index+=1

class CRC_Sim:
    def __init__(self):
        self.Us=[]

    def start(self):
        pass

    def stop(self):
        pass

    def directDrive(self,V1,V2):
        V1= max(min(V1,500),-500)
        V2= max(min(V2,500),-500)
        self.Us.append(np.matrix([V1,V2]).transpose())
    
    def LastU(self):
        if len(self.Us):
            return self.Us[-1]
        return np.matrix([0,0]).transpose()

class KillerSim(KillerRobotCmd):
    def __init__(self,log_queue = None):
        KillerRobotCmd.__init__(self,None,log_queue)
        self.Us = []

    def directDrive(self,V1,V2):
        self.drive(V1,V2)
        self.Us.append(np.matrix([V1,V2]).transpose())

    def LastU(self):
        if len(self.Us):
            return self.Us[-1]
        return np.matrix([0,0]).transpose()


def main():
    
    channel = 'VICON_sawbot'
    r_wheel = 125#mm
    dt = 1.0/5.0
    dt = 0.15

    '''Q should be 1/distance deviation ^2
    R should be 1/ speed deviation^2
    '''

    dist = 20.0 #mm
    ang = 0.25 # radians

    Q = np.diag([1.0/(dist*dist),
                 1.0/(dist*dist),
                 1.0/(ang*ang)])


    R = np.eye(2)
    command_variation = 20.0
    R = np.diag([1/( command_variation * command_variation ),
                 1/( command_variation * command_variation )] )

    maxUc = 20;

    r_circle = 300#mm
    speed = 25 #64
    Xks = circle(r_circle,dt,speed)
    #Xks = straight(1000,1.0/5.0,speed)
    #Xks = loadTraj('../Media/card2-dist5.20-rcut130.00-trajs-0.npy')
    Xks,Uks = TrajToUko(Xks,r_wheel,dt)
    #Xks = Xks[:50]

    lock = Lock()

    start = np.matrix(Xks[0][0:3]).transpose()

    print start

    speedup = 100.0

    sh = StateHolder(lock,start)
    timelock = TickTock()

    runNum = 8
    
    #Setup the logging
    LogQ = Queue(0)
    Log_stopper = Event()
    logname = "Run%i.csv"%runNum
    Log = Logger(logname,Log_stopper,LogQ)

    LogQ2 = Queue(0)
    Log_stopper2 = Event()
    logname2 = "time%i.csv"%runNum
    Log2 = Logger(logname2,Log_stopper2,LogQ2)

    LogQ3 = Queue(0)
    Log_stopper3 = Event()
    logname3 = "cmds%i.csv"%runNum
    Log3 = Logger(logname3,Log_stopper3,LogQ3)

    #setup the controller
    T = 5 #horizon
    RunControl = True
    CC_stopper = Event()
    CRC = KillerSim(LogQ3)  # CRC_Sim()
    CC = Controller(CC_stopper,CRC,sh,Xks,Uks,r_wheel,dt,Q,R,T,maxUc,LogQ,not RunControl,speedup,timelock,LogQ2)

    VSim_stopper = Event()
    #setup the simulator
    VSim = Simulator(VSim_stopper,CRC,sh,Xks,r_wheel,dt,Q,speedup,timelock)
    CRC.start()
    VSim.start()
    #time.sleep(0.005)
    CC.start()
    Log.start()
    Log2.start()
    Log3.start()

    CC.join()
    CRC.stop()
    VSim_stopper.set()
    Log_stopper.set()
    Log_stopper2.set()
    Log_stopper3.set()
    VSim.join()
    Log.join()
    Log2.join()
    Log3.join()
    print 'Done'
    time.sleep(0.05)
    plotCSVRun(logname)
    return 0



if __name__ == "__main__":
    sys.exit(int(main() or 0))