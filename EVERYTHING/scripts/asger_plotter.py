#!/usr/bin/env python
"""
Ploots 

"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import matplotlib as mpl


def findIndex(x, y, X, Y, gridsize):
    indexx, indexy = 0, 0
    valx, valy = abs(x-X[0]-gridsize/2), abs(y-Y[0]-gridsize/2)
    for n in range(0, len(X)):
        if abs(x-X[n]-gridsize/2) < valx:
            indexx = indexx + 1
            valx = abs(x-X[n]-gridsize/2)

    for n in range(0, len(Y)):
        if abs(y-Y[n]-gridsize/2) < valy:
            indexy = indexy + 1
            valy = abs(y-Y[n]-gridsize/2)

    return indexx, indexy


def is_float(element):
    try:
        float(element)
        return True
    except ValueError:
        return False


if __name__ == "__main__":

    f = open('poses-simulation02.txt', "r")

    data02 = ([[float(elem) for elem in row.split(",") if is_float(elem)]
               for row in f.read().split("\n")])

    data02_2 = np.array(data02[1:-1])
    ev_n = 2
    meas02 = data02_2[:, 1][:: ev_n]
    x = data02_2[:, 2][:: ev_n]
    y = data02_2[:, 3][:: ev_n]
    onetothree02 = np.linspace(0, len(meas02), len(meas02))

    f = open('poses-simulation03.txt', "r")
    data03 = ([[float(elem) for elem in row.split(",") if is_float(elem)]
               for row in f.read().split("\n")])

    data03_2 = np.array(data03[1:-1])
    meas03 = data03_2[:, 1][:: ev_n]
    onetothree03 = np.linspace(0, len(meas03), len(meas03))


    f = open('poses-simulation04.txt', "r")
    data04 = ([[float(elem) for elem in row.split(",") if is_float(elem)]
               for row in f.read().split("\n")])

    data04_2 = np.array(data04[1:-1])
    meas04 = data04_2[:, 1][:: ev_n]
    onetothree04 = np.linspace(0, len(meas04), len(meas04))
    #x = [1,2,3, 2,3,1,4]
    #y = [1,2,3,3,1,2,5]
    #meas = [1,2,1,3,2,5,3]

    threshold = 0.01
    meas03_thresh = [meas if meas > threshold else threshold for meas in meas03] 
    meas02_thresh = [meas if meas > threshold else threshold for meas in meas02] 
    meas04_thresh = [meas if meas > threshold else threshold for meas in meas04] 

    plt.rcParams.update({
        "text.usetex": True,
        "font.family": "serif",
        "font.serif": ["Palatino"],
    })


    fig = plt.figure()
    ax1 =  fig.add_subplot(3,1,1)
    plt.plot(onetothree03, meas03)
    plt.plot(onetothree03, meas03_thresh ,"--")
    plt.ylabel(r"Intensity \bigg[$\displaystyle\frac{1}{m^2}$\bigg]")
    plt.xlim([0, 450])
    plt.ylim([0.001, 10])
    ax2 =  fig.add_subplot(3,1,3)
    plt.plot(onetothree02, meas02)
    plt.plot(onetothree02, meas02_thresh ,"--")
    plt.ylabel(r"Intensity \bigg[$\displaystyle\frac{1}{m^2}$\bigg]")
    plt.xlim([0, 450])
    plt.ylim([0.001, 10])
    ax3 =  fig.add_subplot(3,1,2)
    plt.plot(onetothree04, meas04)
    plt.plot(onetothree04, meas04_thresh ,"--")
    plt.ylabel(r"Intensity \bigg[$\displaystyle\frac{1}{m^2}$\bigg]")
    plt.xlim([0, 450])
    plt.ylim([0.001, 10])


    ax1.set_yscale('log')
    ax1.legend([r"Measured data",r"Threshold"])
    ax1.grid(which="major", linestyle='--')
    
    ax2.set_yscale('log')
    ax2.legend([r"Measured data",r"Threshold"])
    ax2.grid(which="major", linestyle='--')
    
    ax3.set_yscale('log')
    ax3.legend([r"Measured data",r"Threshold"])
    ax3.grid(which="major", linestyle='--')
    
    plt.xlabel("Samples")
    #plt.ylabel("Samples")
    plt.tight_layout()

    fig.savefig("asger_plot.png")
    plt.show()
