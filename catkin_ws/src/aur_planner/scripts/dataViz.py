#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal

def dataVis(data, ne_map, scale):
	gaussians = []
	res = 100
	meas_list = []
	for y,x,meas in data:
		s = np.eye(2)*scale
		g = multivariate_normal(mean=(x,y), cov=s)
		gaussians.append(g)
		meas_list.append(meas)

	print(meas_list)

	# create a grid of (x,y) coordinates at which to evaluate the kernels
	x = np.linspace(ne_map[1][0], ne_map[1][1], res)
	y = np.linspace(ne_map[0][1], ne_map[0][0], res)
	xx, yy = np.meshgrid(x,y)
	xxyy = np.stack([xx.ravel(), yy.ravel()]).T

	# evaluate kernels at grid points
	zz = 0
	print(len(data))
	for n in range(len(data)):
		zz = zz+ gaussians[n].pdf(xxyy)*np.sqrt(meas_list[n])

	img = zz.reshape((res,res))
	plt.imshow(img, extent=[ne_map[1][0], ne_map[1][1],ne_map[0][0],ne_map[0][1]])
	plt.show()


def is_float(element):
    try:
        float(element)
        return True
    except ValueError:
        return False


if __name__ == "__main__":

	ne_map = np.array([[-5, 20],[-5,20]])
	SCALE = 3  # increase scale to make larger gaussians
	# Data is = (north, east, meas)
	Data = [(1,1,1), 
			(6,1,2),
			(6,7,3)
			   ] # center points of the gaussians
	f = open('poses.txt',"r")

	#print(f.read().split("\n"))
	#data = np.array([[elem for elem in row.split(",")] for row in f.read().split("\n")])
	
	
	data = ([[float(elem) for elem in row.split(",") if is_float(elem)] for row in f.read().split("\n") ])

	data2 = np.array(data[1:-1])

	meas = data2[:,1]
	x = data2[:,2]
	y = data2[:,3]
	Data = data2[:,1:4]
	print(Data)
	dataVis(Data, ne_map, SCALE)

	