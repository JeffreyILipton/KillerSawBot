from threading import Thread, Lock
from CreateStateEstimator import *
from CreateInterface import *
from CreateModel import *
from TrajectoryTests import *
import scipy
from scipy.optimize import minimize
import sys

import csv

from math import fabs
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

def dBdtheta(th):
    B = np.matrix([[-0.5*sin(th), -0.5*sin(th)],
                   [ 0.5*cos(th),  0.5*cos(th)],
                   [ 0,            0]])
    return B


def diffXs(Xbar,Xtraj):
    DX = (Xbar.reshape((Xbar.size,1))-Xtraj)
    for i in range(4,Xbar.size,5):
        DX[i] = minAngleDif(Xbar[i],Xtraj[i])
    return DX

def makeSuperQ(Q,T):
    diag=[]
    for i in range(0,T):
            diag.append(np.zeros((2,2)))
            diag.append(Q)
    qbar = scipy.sparse.block_diag(diag).todense()
    return qbar

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


class CreateController(Thread):
    def __init__(self,CRC,stateholder,Xks,Uks,ro,dt,Q,R,T,maxU=100,speedup = 1,ticktoc = None, NoControl=False):
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
        self.CRC.start()
        
        self.offset = np.matrix([0,0,0]).transpose()
        
        self.Uos = Uks
        self.Xks = Xks
        #self.Ks = TVLQR(self.Xks, self.Uos, dt, ro, Q, R)
        self.maxU = maxU
        self.index = 0
        self.csvFile = open("Run.csv",'wb')
        self.writer = csv.writer(self.csvFile)

        row=['Time','X_target','Y_target','Angle_target','X_actual','Y_actual','Angle_actual','DX angle','U[0]','U[1]','Uc[0]','Uc[1]']
        self.writer.writerow(row)




        
    def transform(self,X):
        
        th = -self.offset[2,0]
        Rv = np.matrix([[cos(th), -sin(th) ,0],
                       [sin(th), cos(th)  ,0],
                       [0,0,1]])

        th = self.Xks[0][2]
        Rp = np.matrix([[cos(th), -sin(th) ,0],
                       [sin(th), cos(th)  ,0],
                       [0,0,1]]) 


        return Rp.dot(Rv.dot(X-self.offset))+(np.matrix(self.Xks[0][0:3]).transpose())

    def run(self):
        Qbar = makeSuperQ(self.Q,self.T)
        waittime = 0 # self.dt/self.speedup
        Xguess = xtrajMaker(self.Xks,self.Uos,self.T,self.index)
        while True and self.index<( len(self.Uos)-2):

            

            T = min( (len(self.Uos) - self.index-1), self.T)

            tic = time.time()

            # the current State
            X_m = self.holder.GetConfig()
            t = self.holder.getTime()

            #print "state %0.3f,%0.3f,%0.3f"%(s[0],s[1],s[2]) 
            #X = np.matrix([s[0],s[1],s[2]])
            index = self.index
            if(self.index ==0):
                # for first time step set offset and start movement
                self.offset = X_m
                print 'offset:',  self.offset


            X_m = self.transform(X_m)
            Xtraj = xtrajMaker(self.Xks,self.Uos,T,index)
            if (T!= self.T): Qbar = makeSuperQ(self.Q,T)            

            constrains = ({'type':'eq',
               'fun':lambda x: dynamicConstraint(x,X_m,self.dt,self.ro,T),
               'jac': lambda x: dynamicJacobian(x,X_m,self.dt,self.ro,T)})
            

            if self.index !=0:
                Xguess = xGuess(Xguess,XStar.x,self.ro,self.dt,T)
            

            #Xguess
            targetobj = lambda x: obj(x,Xtraj,Qbar)
            targetjac = lambda x: jacobian(x,Xtraj,Qbar)
            XStar = minimize(targetobj,np.squeeze(np.asarray(Xguess)),method='SLSQP',
                                options = {'maxiter':10},
                                #bounds = self.bounds,
                                constraints = constrains,#)#,
                                jac = targetjac)
            U = XStar.x[0:2]

            

            #look out for theta wrap around

                
            Xk = np.matrix(self.Xks[index]).transpose()
            DX = X_m- Xk
            DX[2,0] = minAngleDif(X_m[2,0],self.Xks[index][2])
            Uc = np.squeeze(np.array(self.Uos[self.index]).transpose())

            Udif = U-Uc
            for i in range(0,2):
                if fabs(Udif[i])>self.maxU:
                    U[i] = self.maxU*Udif[i]/fabs(Udif[i])+Uc[i]


            if self.nocontrol: U=Uc

            step = False
            if self.ticktock == None: step=True
            elif self.ticktock.conTick(): 
                step = True
                self.ticktock.setConI(self.index+1)

            toc = time.time()
            time_taken = toc-tic
            print time_taken
            waittime = self.dt-time_taken
            waittime = max(waittime,0)
            time.sleep(waittime/self.speedup)

            if step:
                self.CRC.directDrive(U[0],U[1])

                # add to log
                row = [t]+[Xk[0,0], Xk[1,0],  Xk[2,0] ]+[X_m[0,0], X_m[1,0],  X_m[2,0] ]+[DX[2,0]]+[U[0],U[1]]+[Uc[0],Uc[1]]
                print "I:",self.index

                self.writer.writerow(row)

                self.index +=1
            
            
            

        self.CRC.stop()
        self.csvFile.close()
        print "closed"

def main():
    channel = 'VICON_sawbot'
    r_wheel = 125#mm
    dt = 1.0/5.0




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

    #Xks = loadTraj('../Media/card4-dist5.20-rcut130.00-trajs-0.npy')
    r_circle = 260#mm
    speed = 20 #64
    Xks = circle(r_circle,dt,speed)#loadTraj('../Media/card4-dist5.20-rcut130.00-trajs-0.npy')
    Xks,Uks = TrajToUko(Xks,r_wheel,dt)

    maxU = 15.0


    lock = Lock()

    start = np.matrix(Xks[0][0:3]).transpose()

    sh = StateHolder(lock,np.matrix([0,0,0]).transpose())

    VI = ViconInterface(channel,sh)
    T = 5
    CRC = CreateRobotCmd('/dev/ttyUSB0',Create_OpMode.Full,Create_DriveMode.Direct)
    CC = CreateController(CRC,sh,Xks,Uks,r_wheel,dt,Q,R,T,maxU,NoControl=False)


    
    VI.start()
    time.sleep(0.05)
    CC.start()

    CC.join()
    #VI.join()

    print "Done"
    plotCSVRun()

if __name__ == "__main__":
    sys.exit(int(main() or 0))
