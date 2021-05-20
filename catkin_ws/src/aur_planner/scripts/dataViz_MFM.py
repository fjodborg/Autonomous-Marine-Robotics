#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import matplotlib as mpl
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from sys import argv



def fun(x, y):
    return x**2 + y

def is_float(element):
	try:
		float(element)
		return True
	except ValueError:
		return False

if __name__ == "__main__":

	f = open('poses.txt',"r")
	
	data = ([[float(elem) for elem in row.split(",") if is_float(elem)] for row in f.read().split("\n") ])
	data2 = np.array(data[1:-1])
	measments = data2[:,1]
	xdata = data2[:,2]
	ydata = data2[:,3]
	
	#x,y,z = np.loadtxt('poses.txt', unpack=True)

	#x,y,measments = np.loadtxt('poses.txt',dtype = float, skiprows=1, usecols=(1, 2,3))
	x,y,measments = np.loadtxt('poses.txt', dtype=float, delimiter=",", converters=None, skiprows=1, usecols=(1,2,3), unpack=True, ndmin=0, encoding='bytes', max_rows=None)


	fig = plt.figure()
	ax = Axes3D(fig)
	surf = ax.pcolormesh(x, y, measments, cmap=cm.jet, linewidth=0.1)
	#fig.colorbar(surf, shrink=0.5, aspect=5)
	#plt.savefig('teste.pdf')
	plt.show()