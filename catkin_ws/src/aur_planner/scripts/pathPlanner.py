"""

Pathplanner

"""


import numpy as np
import matplotlib.pyplot as plt
from missionDesigner import missionDesigner

class pathPlanner():

	def __init__(self, meas_thres, wg, wl, circ_acc, circ_acc_l, ng, nl):
		self.meas_thres = meas_thres
		self.wg, self.wl = wg, wl
		self.circ_acc, self.circ_acc_l = circ_acc, circ_acc_l
		self.ng, self.nl = ng, nl

		# Init Output Waypoint
		self.Waypoint = wg[:,0]

		# Init Internal variables
		self.index_g, self.index_l, self.x_0, self.g_l_flag = 0, 0, 0.0, 0

	def stateMachine(self, x, meas):
		self.updateVariables(x)
		self.Waypoint = wg[:,self.index_g]
		print(self.index_l)
		if (meas >= self.meas_thres and self.g_l_flag == 0):
			self.g_l_flag = 1
			self.x_0 = x
			print("Her")

		if self.g_l_flag == 1:
			self.Waypoint = wl[:,self.index_l]+self.x_0.T

			circ = (wl[0,self.index_l]-x[0]+self.x_0[0])**2+(wl[1,self.index_l]-x[1]+self.x_0[1])**2

			if circ <= self.circ_acc_l**2:
				self.index_l = self.index_l + 1

		return self.Waypoint

	def updateVariables(self, x):
		if self.index_g > self.ng:
			self.index_g = 0
		if self.index_l > self.nl:
			self.index_l, self.g_l_flag = 0, 0

		circ = (wg[0,self.index_g]-x[0])**2+(wg[1,self.index_g]-x[1])**2

		if circ <= self.circ_acc**2:
			self.index_g = self.index_g + 1


	def show_param(self):
	        print(vars(self)) # Show all attributes with values


if __name__ == "__main__":

	ng = 4
	nl = 3

	MD = missionDesigner(northg=2.0, eastg=ng, ng=ng, northl=0.5, eastl=1.5, nl=nl, northg0=1.0, eastg0=0.0)
	wg, wl = MD.return_waypoints()
	pp = pathPlanner(meas_thres = 5, wg = wg, wl = wl, circ_acc = 2, circ_acc_l = 0.1, ng = ng, nl = nl)

	x = np.array([[1.5,1]]).T
	print(pp.stateMachine(x, 4))
	meas = 5

	print(pp.stateMachine(x, meas))
	print(pp.stateMachine(x, meas))
	print(pp.stateMachine(x, meas))