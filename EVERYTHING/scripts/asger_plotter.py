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
    #x = [1,2,3, 2,3,1,4]
    #y = [1,2,3,3,1,2,5]
    #meas = [1,2,1,3,2,5,3]

    fig = plt.figure()
    plt.plot(onetothree02, meas02)
    fig = plt.figure()
    plt.plot(onetothree03, meas03)
    plt.show()
