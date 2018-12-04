from math import *
import numpy as np

'''
let the vehicle have two wheels with speeds U1 and U2. lets U1 be the one on the left and U2 the one on the right when looking form above with the front of hte vehicle
            X_car
           / \
            |
            |
------|<----o---->|-------> Y_car
     U1     |    U2
            |
            |
           \ /


V_cm = 0.5*(U1+U2) = sqrt( X_dot^2 + Y_dot^2 ) 

Theta_dot = 1/ro * 1/2 * (U2-U1)

X_dot = BU
 _         _       _                              _     _  _
| X_dot     |     | 1/2*cos(theta), 1/2*cos(theta) | * | U1 |
| Y_dot     |   = | 1/2*sin(theta), 1/2*sin(theta) |   |_U2_|
|_Theta_dot_|     |_-1/(2*ro)     , 1/(2*ro)      _|

X_k+1 = I+Bu

Inverting V_cm and theta_dot eq we get

U1 = V-R*theta_dot
U2 = V+R*theta_dot
'''

def minAngleDif(x,y):
     a = atan2(sin(x-y), cos(x-y))
     #if(a<0):a = 2*pi-a
     return a


def V(Xk,Xkp1,dt):
    vm = [ (Xkp1[0]-Xk[0])/dt  , ((Xkp1[1]-Xk[1])/dt) ]

    return vm

def UkFromXkandXkplusone(Xk,Xkp1,ro,dt):
    #V = sqrt(  ((Xkp1[0]-Xk[0])/dt)**2  + ((Xkp1[1]-Xk[1])/dt)**2 )
    #http://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    #d_theta = minAngleDif(Xkp1[2],Xk[2])
    #thetadot = d_theta/dt
    
    DX = Xkp1-Xk
    DX[2] = minAngleDif(Xkp1[2],Xk[2])
    Binv = 1.0/dt*PseudoBInv(Xk[2],ro)

    #Uk = [(V-ro*thetadot),(V+ro*thetadot)]
    #Uk = np.array(Uk) 

    #Uk = np.array([[(V+ro*thetadot)],
    #               [(V-ro*thetadot)]])
    
    Uk = Binv.dot(DX)
    G,Mo = MotorGainAndOffset()
    GI = np.linalg.inv(G)
    Uc = GI.dot(Uk.transpose()-Mo)

    return  Uc.transpose()

def sameSign(a,b):
    if (a==0 or b==0) and (a!=0 or b!=0): return False
    return ((a<0) == (b<0))

def dotProduct(v1,v2):
    return v1[0]*v2[0]+v1[1]*v2[1]

def TrajToUko(Xks,ro,dt,):
    Ukos = []
    prev_v = [0,0]
    new_xks = []
    for i in range(0,len(Xks)-1):
        Xk = Xks[i]
        Xkp1 = Xks[i+1]
        v = V(Xk,Xkp1,dt)
        Uko = np.array(UkFromXkandXkplusone(Xk,Xkp1,ro,dt))
        npts=1
        if (dotProduct(prev_v,v) <= 0):
            delay = DelayModel(sqrt(dotProduct(v,v)))
            npts = int(delay/dt)
        for j in range(0,npts):
            new_xks.append(Xk)
            Ukos.append(Uko)
        prev_v = v
    return new_xks,Ukos


def B(th,r0):
    '''returns the B matrix
       this expects a 1d array for X
    '''
    #g = 0.7964
    B = np.matrix([[.5*cos(th), .5*cos(th)],[.5*sin(th), .5*sin(th)],[1.0/(2.0*r0), -1.0/(2.0*r0)]])
    return B

def PseudoBInv(th,ro):
    Binv = np.matrix([[cos(th), sin(th),ro],[cos(th), sin(th),-ro]])
    return Binv
    
def A(dt):
    Ia = np.mat(np.eye(3))
    #Dt = dt*Ia
    #Za = np.mat(np.zeros((3,3)))
    #A = np.bmat([[Ia,Dt],[Za,Za]])
    return Ia


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



def DelayModel(speed):
    if speed < 17.5: return 2.0
   
    Mslope = -0.021
    MIntercept = 1.377
    delay = Mslope*speed+MIntercept
    return delay


def MotorGainAndOffset():
    G = np.diag([0.855,0.7337])
    V = np.mat([[-1.8889],[-0.6113]])
    return G,V