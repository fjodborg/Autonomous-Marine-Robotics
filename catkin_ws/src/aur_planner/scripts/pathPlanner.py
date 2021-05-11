#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import matplotlib.pyplot as plt
from missionDesigner import missionDesigner
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Illuminance  # From the sensor
from uuv_control_msgs.srv import GoTo
from uuv_control_msgs.msg import Waypoint


class pathPlanner():

    def __init__(self, meas_thres, wg, wl, circ_acc, circ_acc_l, ng, nl):
        self.meas_thres = meas_thres
        self.wg, self.wl = wg, wl
        self.circ_acc, self.circ_acc_l = circ_acc, circ_acc_l
        self.ng, self.nl = ng, nl

        # Init Output Waypoint
        self.Waypoint = wg[:, 0]

        # Init Internal variables
        self.index_g, self.index_l, self.x_0, self.g_l_flag = 0, 0, 0.0, 0

    def stateMachine(self, pos, meas):
        self.updateVariables(pos)
        self.Waypoint = wg[:, self.index_g]
        print("Global index: %d, Local: %d, Local_flag %d" %
              (self.index_g, self.index_l, self.g_l_flag))

        if (meas >= self.meas_thres and self.g_l_flag == 0):
            self.g_l_flag = 1
            self.x_0 = pos

        if self.g_l_flag == 1:
            self.Waypoint = wl[:, self.index_l]+self.x_0.T

            circ = (wl[0, self.index_l]-pos[0]+self.x_0[0])**2 + \
                (wl[1, self.index_l]-pos[1]+self.x_0[1])**2

            if circ <= self.circ_acc_l**2:
                self.index_l = self.index_l + 1
        return self.Waypoint

    def updateVariables(self, x):
        # x - positions of robot, [N, E]
        circ = (wg[0, self.index_g]-x[0])**2+(wg[1, self.index_g]-x[1])**2

        if circ <= self.circ_acc**2:
            self.index_g = self.index_g + 1

        if self.index_g >= self.ng:
            self.index_g = 0
        if self.index_l >= self.nl:
            self.index_l, self.g_l_flag = 0, 0

    def show_param(self):
        print(vars(self))  # Show all attributes with values


class Subscriber():
    def __init__(self, pathPlanner):
        self.PP = pathPlanner  # Access to pp class

        self.pose = None
        self.simulation_flag = True  # are we in a simulation?
        self.max_speed = 100.0  # Unit unknown, used twice in the same message
        self.interpolator = ''  # Effect unknown
        self.WP = Waypoint()
        self.WP.max_forward_speed = self.max_speed
        self.WP.radius_of_acceptance = 0.5  # [m]

        rospy.wait_for_service('/bluerov2/go_to')
        self.goto_srv = rospy.ServiceProxy('/bluerov2/go_to', GoTo)
        # self.pub = rospy.Publisher('bluerov2/our_sensor', GoTo, queue_size=1) # Service

        rospy.Subscriber("/bluerov2/our_sensor", Illuminance,
                         self.measurement_callback)
        if (self.simulation_flag):
            rospy.Subscriber("/bluerov2/pose_gt", Odometry,
                             self.pose_callback_simulation)

    def pose_callback_simulation(self, odom):
        # Do this when something is published to intilized topic
        self.odom = odom
        self.pose = self.odom.pose.pose  # Extract what we need
        self.pos = np.array(
            [self.pose.position.x, self.pose.position.y])  # homemade FTW
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose.position)

    def measurement_callback(self, measurement):
        if self.pose == None:
            return
        self.meas = measurement.illuminance
        WP_simple = pp.stateMachine(self.pos, self.meas)
        print(WP_simple)
        self.WP.point.x = WP_simple[0]
        self.WP.point.y = WP_simple[1]
        srv_return = self.goto_srv(self.WP, self.max_speed, self.interpolator)
        print(srv_return)


if __name__ == "__main__":

    rospy.init_node('our_path_planner', anonymous=True)

    ng = 4  # Points in the global plan
    nl = 4  # Points in the local plan
    MD = missionDesigner(northg=10.0, eastg=10.0, ng=ng, northl=1.5,
                         eastl=1.5, nl=nl, northg0=0.0, eastg0=0.0)
    wg, wl = MD.return_waypoints()
    pp = pathPlanner(meas_thres=0.1, wg=wg, wl=wl, circ_acc=0.2,
                     circ_acc_l=0.1, ng=ng, nl=nl)

    Sub = Subscriber(pp)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # sensor.illuminance += 0.1
        # rospy.loginfo(sensor.illuminance)
        rate.sleep()

    # x = np.array([[1.5,1]]).T
    # print(pp.stateMachine(x, 4))
    # meas = 5

    # print(pp.stateMachine(x, meas))
    # print(pp.stateMachine(x, meas))
    # print(pp.stateMachine(x, meas))
    rospy.spin()
