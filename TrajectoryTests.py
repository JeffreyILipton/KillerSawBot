import numpy as np
from math import *
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys
import pylab as pl
from Model import *

def circle(r,dt,speed):
    # mm,s,mm/s
    dist = speed*dt
    c = pi*r*2
    n = int(round(c/dist))
    Xks=[]
    for i in range(0,n+1):
        x = r*cos(2*pi/n*i)
        y = r*sin(2*pi/n*i)
        theta = (2*pi/n*i+pi/2.0)%(2*pi)
        Xk = [x,y,theta]



        Xks.append(Xk)

    return np.array(Xks)


def straight(l,dt,speed):
    dist_per_step = speed*dt
    n = int(round(l/dist_per_step))
    Xks = []
    for i in range(0,n+1):
        theta = pi/4
        r = i*dist_per_step
        x = r*cos(theta)
        y = r*sin(theta)
        Xk = [x,y,theta]
        Xks.append(Xk)
    return np.array(Xks)
    
def forandBack(l,dt,speed):
    forward = straight(l,dt,speed)
    back = forward[:0:-1]
    return np.vstack((forward,back))

def loadTraj(file):
    return np.load(file)


def plotUandX(xs,UKs,dt):
    fig,ax=pl.subplots()
    X = np.array(xs)

    plt.figure(1)
    plt.subplot(111)
    plt.plot(X[:,0],X[:,1],'bx-')
    plt.axis('equal')

    U = np.vstack(UKs)
    
    T = np.linspace(0,X.shape[0]*dt,U.shape[0])
    plt.figure(2)
    plt.subplot(211)
    plt.plot(T,U[:,0],'bo-')
    plt.plot(T,U[:,1],'gx-')
    plt.subplot(212)
    T = np.linspace(0,X.shape[0]*dt,X.shape[0])
    plt.plot(T,X[:,0],'go-')
    plt.plot(T,X[:,1],'bx-')
    plt.show()

def vectorsFromOPt(qpt):
    pt = [qpt[0],qpt[1]]
    T=[]
    if(len(qpt)>2): 
        if (qpt[2] !=None):
            T = [cos(qpt[2]),sin(qpt[2]) ]
    else: T = None
    return [pt,T]

def plotTraj(xs):
    quivers = []
    for i in range(0,xs.shape[0]):
        pt, vec = vectorsFromOPt(xs[i])
        pt = pt+vec
        quivers.append(pt)
    q = np.array(quivers)
    X,Y,U,V = zip(*q)

    fig,ax=pl.subplots()
    ax.quiver(X,Y,U,V,angles='xy',scale_units='xy',scale=10, color='k')
    ax.autoscale()
    ax.margins(0.1)
    ax.axis('equal')
    plt.axis('equal')
    plt.show()





def main():
    r=254.0 # 10 in 254cm 
    dt=1.0/5.0
    speed = 26.5988178 #mm / s
    #xs = circle(r,dt,speed)

    #xs = straight(100,1.0/5.0,speed)

    xs = loadTraj('../Media/Cardiod-rc300.00-spacing5.00-rcut127.00-trajs-0.npy')
    xs = forandBack(100,dt,speed)
    plotTraj(xs)
    
    r_wheel = 125
    XKs,UKs = TrajToUko(xs,r_wheel,dt)

    plotUandX(XKs,UKs,dt)
    
    return 0




if __name__ == "__main__":
    sys.exit(int(main() or 0))



