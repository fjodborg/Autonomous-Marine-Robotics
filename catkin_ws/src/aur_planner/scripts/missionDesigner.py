"""
Mission designer

"""


import numpy as np
import matplotlib.pyplot as plt

class missionDesigner():

	def __init__(self, northg, eastg, ng, northl, eastl, nl, northg0, eastg0):
		self.northg, self.eastg, self.ng, \
		 self.northl, self.eastl, self.nl, \
		 self.northg0, self.eastg0 = northg, eastg, ng, northl, eastl, nl, northg0, eastg0

		self.waypoints_g, self.waypoints_l = np.zeros((2,int(self.ng))), np.zeros((2,int(self.nl)));

		# Variables for global wapoint generator
		step_g_east = (self.eastg/(self.ng+(self.ng%2)))*2
		step_g_east = step_g_east+step_g_east/((ng+(self.ng%2))/2-1)

		north_val = self.northg0
		east_val = self.eastg0

		# Create global waypoint
		for n in range(1, int(self.ng+1)):
			self.waypoints_g[:,n-1] = np.hstack((north_val, east_val))
			north_val = north_val + self.northg*(n%2)*(-1)**((n**2+n+2)/2)
			east_val = east_val+step_g_east*((n+1)%2)

		# Variables for local wapoint generator
		step_l_east = (self.eastl/(self.nl+(self.nl%2)))*2
		step_l_east = step_l_east+step_l_east/((nl+(self.nl%2))/2-1)

		north_val = 0.0
		east_val = 0.0

		# Create local waypoint
		for n in range(1, int(self.nl+1)):
			self.waypoints_l[:,n-1] = np.hstack((north_val, east_val))
			north_val = north_val + self.northl*(n%2)*(-1)**((n**2+n+2)/2)
			east_val = east_val+step_l_east*((n+1)%2)

	def return_waypoints(self):
		return self.waypoints_g, self.waypoints_l
	def show_param(self):
	        print(vars(self)) # Show all attributes with values



if __name__ == "__main__":


	MD = missionDesigner(northg=2.0, eastg=4.0, ng=10.0, northl=0.5, eastl=1.5, nl=3.0, northg0=1.0, eastg0=0.0)
	wg, wl = MD.return_waypoints()

	print("Global waypoints =\n {}\n Local waypoints =\n {}".format(wg,wl))

	plt.figure(1)
	plt.subplot(211)
	plt.plot(wg[1],wg[0], 'o', color='black');
	plt.title("Global waypoints")

	plt.subplot(212)
	plt.plot(wl[1],wl[0], 'o', color='black');
	plt.title("Local waypoints")
	plt.show()





