#!/usr/bin/python
import csv
import numpy as np
from numpy import genfromtxt
import matplotlib
import matplotlib.pyplot as plt
import sys
from CreateModel import minAngleDif
from scipy.stats import linregress


def AnalyseFile(file='Run.csv'):
    data = genfromtxt(file, delimiter=',')
    print data.shape
    headings = data[0, :]
    data = np.delete(data, 0, 0)
    print data.shape

    # convert T from ms since epoch to seconds since start
    #T = data[:,0]
    #T = T-T[0]
    #T = T/(1e6)
    # data[:,0]=T

    speeds = np.unique(data[:, 1])
    speeds = np.delete(speeds, 0, 0)
    analysis = np.zeros((speeds.size, 4))
    for i in range(0, speeds.size):
        speed = speeds[i]
        analysis[i, :] = analyzeSpeed(speed, data, True)

    Uslope, Uintercept, U_r_value, U_p_value, U_std_err = linregress(
        analysis[:, 0], analysis[:, 2])
    calced = Uslope * analysis[:, 0] + Uintercept

    plt.figure(1)
    plt.plot(analysis[:, 0], analysis[:, 1], '-')
    plt.title("cmd vs delay")
    quicker = analysis[np.where(analysis[:, 1] < 1.2)]
    Mslope, Mintercept, M_r_value, M_p_value, M_std_err = linregress(
        quicker[:, 0], quicker[:, 1])
    print "Mslope: ", Mslope, "\tr:", M_r_value, "\t stdErr: ", M_std_err
    print "Mintercept: ", Mintercept
    MCalced = Mslope * analysis[:, 0] + Mintercept
    plt.plot(analysis[:, 0], MCalced, '--')

    plt.figure(2)
    plt.plot(analysis[:, 0], analysis[:, 2], '-')
    plt.plot(analysis[:, 0], calced, '--')
    plt.title("cmd vs Actual speed")

    print "Uslope: ", Uslope, "\tr:", U_r_value, "\t stdErr: ", U_std_err
    print "Uintercept", Uintercept

    plt.figure(3)

    ro = 125  # mm

    Tslope, Tintercept, T_r, T_p, T_err = linregress(
        analysis[:, 0], analysis[:, 3])
    Tcalc = Tslope * analysis[:, 0] + Tintercept

    plt.plot(analysis[:, 0], analysis[:, 3], '-')
    plt.plot(analysis[:, 0], Tcalc, '--')
    plt.title("$\Theta% vs cmd speed")
    print "Tslope: ", Tslope, "\tr:", T_r, "\t stdErr: ", T_err
    print "Tintercept", Tintercept

    S = np.mat([[Uslope], [Tslope]])
    O = np.mat([[Uintercept], [Tintercept]])
    alphaInv = np.mat([[1, ro], [1, -ro]])
    G = alphaInv.dot(S)
    V = alphaInv.dot(O)
    print "G:"
    print G
    print "V:"
    print V

    plt.show()


def analyzeSpeed(speed, data, doplot=False):
    section = data[np.where(data[:, 1] == speed)]
    To = section[0, 0]
    Xo = section[0, 2]
    Yo = section[0, 3]
    section[:, 0] = (section[:, 0] - To) / (1e6)

    section[:, 2] = section[:, 2] - Xo
    section[:, 3] = section[:, 3] - Yo
    """
    # bug: this only considers one axis so it's kinda screwy
    moving = section[np.where(np.abs(section[:,3])>2.0)]

    D = np.sqrt(np.square(moving[:,2])+np.square(moving[:,3]));
    # the following fix is hella ghetto but works i guess lol
    
    """
    D = np.sqrt(np.square(section[:, 2]) + np.square(section[:, 3]))
    moving = section[np.where(D[:] > 2.0)]
    D = np.sqrt(np.square(moving[:, 2]) + np.square(moving[:, 3]))
    timeToStart = moving[0, 0]
    slope, intercept, r_value, p_value, std_err = linregress(moving[:, 0], D)

    ThetaRate, ThetaIntercept, Theta_r, Theta_p, Theta_err = linregress(
        moving[:, 0], moving[:, 4])

    if doplot:
        plt.figure(1)
        plt.subplot(311)
        plt.plot(moving[:, 0], D, '-')
        plt.title("D vs T")
        plt.subplot(312)
        plt.plot(moving[:, 2], moving[:, 3], '-')
        plt.title("Y vs X")
        plt.subplot(313)
        plt.plot(moving[:, 0], moving[:, 4], '-')
        plt.plot(moving[:, 0], ThetaRate * moving[:, 0] + ThetaIntercept, '--')
        plt.title("Theta vs T")
        plt.show()

    print "cmd: ", speed, "\tactual:", slope, "\tr:", r_value
    return [speed, timeToStart, slope, ThetaRate]


def main():
    AnalyseFile()


if __name__ == "__main__":
    sys.exit(int(main() or 0))
