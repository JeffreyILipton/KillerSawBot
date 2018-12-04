import numpy as np

def transformToTrajFrame(offset,Xtraj0,X):
        
    th = -offset[2,0]
    Rv = np.matrix([[cos(th), -sin(th) ,0],
                    [sin(th), cos(th)  ,0],
                    [0,0,1]])

    th = Xtraj0[2]
    Rp = np.matrix([[cos(th), -sin(th) ,0],
                    [sin(th), cos(th)  ,0],
                    [0,0,1]]) 

    Xt0 = np.matrix(Xtraj0[0:3]).transpose()

    return Rp.dot(Rv.dot(X-offset))+(Xt0)


def transformToViconFrame(Xinitial,Xtraj0,Xtraj):
        
    th = Xinitial[2,0]
    Rv = np.matrix([[cos(th), -sin(th) ,0],
                    [sin(th), cos(th)  ,0],
                    [0,0,1]])

    th = -Xtraj0[2]
    Rp = np.matrix([[cos(th), -sin(th) ,0],
                    [sin(th), cos(th)  ,0],
                    [0,0,1]]) 

    Xt0 = np.matrix(Xtraj0[0:3]).transpose()
    return Rv.dot(Rp.dot(Xtraj-Xt0))+Xinitial