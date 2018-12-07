from threading import Thread, Lock, Event
from StateEstimator import *
from CreateInterface import *
from Model import *
from Simulator import TickTock

import scipy
from scipy.optimize import minimize
import sys

#import csv

from math import fabs


def obj(Xbar,Xtraj,Qbar):
        DX = diffXs(Xbar,Xtraj)
        #DX = (Xbar.reshape((Xbar.size,1))-Xtraj)
        J = 0.5*DX.T.dot(Qbar).dot(DX)
        j = float(J[0,0]) 
        return j

def jacobian(Xbar,Xtraj,Qbar):
    DX = diffXs(Xbar,Xtraj)
    jac = DX.T.dot(Qbar)
    njac = np.squeeze(np.asarray(jac))
    return njac
    
def dynamicConstraint(Xbar,Xnow,dt,ro,T):
    G,V = MotorGainAndOffset()
    C = np.zeros((3*(T),1))
    U1 = Xbar[0:2].reshape((2,1))
    X2 = Xbar[2:5].reshape((3,1))
    Umod = G.dot(U1)+V
    C[0:3] = X2-Xnow -  dt*B(Xnow[2],ro).dot(Umod)

    for i in range(1,T):
        Xkm1 = Xbar[5*(i-1)+2    : 5*(i-1)+2+3].reshape((3,1))
        Ukm1 = Xbar[5*(i-1)+3+2  : 5*(i-1)+4+3].reshape((2,1))
        Xk =   Xbar[5*(i ) +2    : 5*(i  )+2+3].reshape((3,1))
        Umod = G.dot(Ukm1)+V
        diff = Xk - Xkm1 - dt*B(Xkm1[2],ro).dot(Umod)
        C[3*i:3*i+3] = diff
    Cturned = np.squeeze(np.asarray(C))
    return Cturned

def subBlock(Xbar,dt,ro,i):
    U1 = Xbar[5*(i)       :5*(i)+2].reshape((2,1))
    x1 = Xbar[5*(i)-3     :5*(i)].reshape((3,1))

    

    G,V = MotorGainAndOffset()
    Umod = G.dot(U1)+V
    quack = dt*dBdtheta(x1[2]).dot(Umod)
    crack = np.zeros((3,3))
    crack[:,2] = np.squeeze(quack)
    Bk = B(x1[2],ro)
    
    dC1dx1 = -np.eye(3) - crack
    dC1du1 = -dt*Bk.dot(G)
    dC1dx2 = np.eye(3)
    return np.asarray(np.bmat([dC1dx1,dC1du1,dC1dx2]))

def dynamicJacobian(Xbar,Xnow,dt,ro,T):
    Cj = np.zeros((3*(T),5*T))

    Bk = B(Xnow[2],ro)
    G,V = MotorGainAndOffset()
    U1 = Xbar[0:2].reshape((2,1))
    Umod = G.dot(U1)+V
    dC1du1 = -dt*Bk.dot(G)
    dC1dx2 = np.eye(3)
    
    Cj[0:3,0:5]=np.bmat([dC1du1,dC1dx2])

    for i in range(1,T):
        block = subBlock(Xbar,dt,ro,i)
        Cj[3*i:3*i+3,5*i-3:5*i+5] = block

    djacobian = np.squeeze(np.asarray(Cj))
    return djacobian

def bounds(T,Uos,Umaxs):
    b = []
    for i in range(0,T):
        b.append((None,None))
        b.append((None,None))
        b.append((None,None))
        b.append((Uos[i,0]-Umax[0],Uos[i,0]+Umax[0]))
        b.append((Uos[i,1]-Umax[1],Uos[i,1]+Umax[1]))
    return b


def xtrajMaker(Xks,Uks,T,index):
    xtraj = np.zeros((5*T,1))
    for i in range(0,T):
        xtraj[0+5*i:2+5*i] = Uks[index+i].transpose()
        xtraj[2+5*i:5+5*i] = Xks[index+1+i].reshape((3,1))#.transpose()
    return xtraj

def xGuess(Xguess,Xstar,ro,dt,T):
    if T >= Xstar.size/5.0:
        Xguess[0:-5] = Xstar[5:].reshape((Xstar.size-5,1))
        ulast = Xstar[-5:-3]
        xlast = Xstar[-3:]
        xnew = xlast+dt*B(xlast[2],ro).dot(ulast)
        Xguess[-5:-3] = ulast.reshape((2,1))
        Xguess[-3:] = xnew.T
    else:
        Xguess = Xstar[5:5*T+5]
    return Xguess


def MPC(dt,ro,T,Xtraj,Qbar,Xguess,X_m):
    constrains = ({'type':'eq',
        'fun':lambda x: dynamicConstraint(x,X_m,dt,ro,T),
        'jac': lambda x: dynamicJacobian(x,X_m,dt,ro,T)})

    #Set the objective function and its jacobian
    targetobj = lambda x: obj(x,Xtraj,Qbar)
    targetjac = lambda x: jacobian(x,Xtraj,Qbar)

    XStar = minimize(targetobj,np.squeeze(np.asarray(Xguess)),method='SLSQP',
                        options = {'maxiter':10},
                        #bounds = self.bounds,
                        constraints = constrains,#)#,
                        jac = targetjac)
    U = XStar.x[0:2]
    return U,XStar.x

def thresholdU(U,Uc,maxU):
    Udif = U-Uc
    U_clean = np.array(U)
    for i in range(0,2):
        if fabs(Udif[i])>maxU:
            U_clean[i] = maxU*Udif[i]/fabs(Udif[i])+Uc[i]
    return U_clean

class Controller(Thread):
    def __init__(self,stopper,CRC,stateholder,Xks,Uks,ro,dt,Q,R,T,maxU=100,StateQueue=None,NoControl=False,speedup = 1,ticktoc = None, SimQ = None):
        Thread.__init__(self)
        self.nocontrol = NoControl
        self.speedup = speedup
        self.ticktock = ticktoc
        self.CRC = CRC
        self.holder = stateholder
        self.dt = dt
        self.T  =T
        self.Q = Q
        self.ro = ro
        self.stopper = stopper
        self.state_queue = StateQueue
        self.simq = SimQ
        
      
        
        self.Uos = Uks
        self.Xks = Xks
        self.maxU = maxU
        self.index = 0


        
        if type(self.state_queue)!=type(None):
            row=['Time','X_target','Y_target','Angle_target','X_actual','Y_actual','Angle_actual','DX angle','U[0]','U[1]','Uc[0]','Uc[1]']
            self.state_queue.put(row)
        
        if type(SimQ) != type(None):
            row = ["time","dt","speedup"]
            self.simq.put(row)


        


    def run(self):
        waittime = 0 
        Qbar = makeSuperQ(self.Q,self.T)
        while (self.index<( len(self.Uos)-2)) and (not self.stopper.is_set()) :

            tic = time.time()

            # the current State
            X_m = self.holder.GetConfig()
            t = self.holder.getTime()

 
            Uc = np.squeeze(np.array(self.Uos[self.index]).transpose())

            # This is where the control happens 
            if not self.nocontrol:
                #Set the Horizon
                T = min( (len(self.Uos) - self.index-1), self.T)

                if self.index ==0:
                    Xguess = xtrajMaker(self.Xks,self.Uos,self.T,self.index)
                else:
                    Xguess = xGuess(Xguess,XStar,self.ro,self.dt,T)

                if (T!= self.T): Qbar = makeSuperQ(self.Q,T)            
            
                Xtraj = xtrajMaker(self.Xks,self.Uos,T,self.index)

                # Run MPC
                U_init, XStar = MPC(self.dt,self.ro,T,Xtraj,Qbar,Xguess,X_m)
                # Threshold results
                U = thresholdU(U_init,Uc,self.maxU)
            else:        
                U=Uc
            # Threshold difference between  expected and actual



            # blocking if using simulator
            step = False
            if self.ticktock == None: step=True
            elif self.ticktock.conTick(): 
                step = True
                self.ticktock.setConI(self.index+1)

            # wait for time needed
            toc = time.time()
            time_taken = toc-tic
            print time_taken
            waittime = self.dt-time_taken
            waittime = max(waittime,0)
            time.sleep(waittime/self.speedup)

            if step:
                self.CRC.directDrive(U[0],U[1])

            if type(self.simq) != type(None):
                row = [time_taken,self.dt,self.speedup]
                self.simq.put(row)

            if type(self.state_queue)!=type(None):
                
                # add to log
                Xk = np.matrix(self.Xks[self.index]).transpose()
                DX = X_m- Xk
                #look out for theta wrap around                
                DX[2,0] = minAngleDif(X_m[2,0],self.Xks[self.index][2])
                
                row = [t]+[Xk[0,0], Xk[1,0],  Xk[2,0] ]+[X_m[0,0], X_m[1,0],  X_m[2,0] ]+[DX[2,0]]+[U[0],U[1]]+[Uc[0],Uc[1]]
                self.state_queue.put(row)

            print "I:",self.index
            self.index +=1
               
                
            
            
            
        print "Control Done"

def main():


    #setup controller infor
    channel = 'VICON_sawbot'
    r_wheel = 125#mm
    dt = 1.0/5.0
    
    maxU = 15.0


    '''Q should be 1/distance deviation ^2
    R should be 1/ speed deviation^2
    '''
    dist = 20.0 #mm
    ang = 0.5 # radians

    Q = np.diag([1.0/(dist*dist),
                 1.0/(dist*dist),
                 1.0/(ang*ang)])


    R = np.eye(2)
    command_variation = 10.0
    R = np.diag([1/( command_variation * command_variation ),
                 1/( command_variation * command_variation )] )



    # Setup the Trajectory
    #Xks = loadTraj('../Media/card4-dist5.20-rcut130.00-trajs-0.npy')
    r_circle = 260#mm
    speed = 20 #64
    Xks = circle(r_circle,dt,speed)#loadTraj('../Media/card4-dist5.20-rcut130.00-trajs-0.npy')
    Xks,Uks = TrajToUko(Xks,r_wheel,dt)

    # Setup the Robot interface
    CRC = CreateRobotCmd('/dev/ttyUSB0',Create_OpMode.Full,Create_DriveMode.Direct)

    # setup the Vicon interface
    lock = Lock()

    sh = StateHolder(lock,np.matrix([0,0,0]).transpose())

    VI_stopper = Event() # https://christopherdavis.me/blog/threading-basics.html
    VI = ViconInterface(VI_stopper,channel,sh)
    
    VI.start()
    time.sleep(0.05)

    # Get initial position

    Xinitial = sh.GetConfig()
    print 'X_initial:',  Xinitial
    
    # transform to start at robot current position. 
    Xtraj0 = Xks[0]
    Xks = [transformToViconFrame(Xinitial,Xtraj0,Xtraj) for Xtraj in Xks]


    # make the Controller for the trajectory
    T = 5
    CC_stopper = Event()
    LogQ = Queue(0)

    CC = Controller(CC_stopper,CRC,sh,Xks,Uks,r_wheel,dt,Q,R,T,maxU,LogQ,NoControl=False)
    CRC.start() # this must come before CC.start()


    #Setup the logging
    Log_stopper = Event()
    logname = "Run1.csv"
    Log = Logger(logname,Log_stopper,LogQ)

    # Start log and controll
    CC.start()
    Log.start()
    
    # Wait for the control to be done and stop logging and vicon
    CC.join()
    Log_stopper.set()
    Log.join()
    VI_stopper.set()
    VI.join()

    print "Done"
    # Make plots!
    plotCSVRun(logname)

if __name__ == "__main__":
    from plotRun import *
    from Logging import *
    from TrajectoryTests import *
    from Trajectory import *

    sys.exit(int(main() or 0))
