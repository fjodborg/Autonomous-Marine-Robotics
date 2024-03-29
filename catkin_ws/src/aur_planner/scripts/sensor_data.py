#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Illuminance
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
from enum import Enum
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
buffer_size = 3


class Method(Enum):
    INV_EUC = 1
    INTENSITY = 2


method = Method.INTENSITY


class Log():
    def __init__(self):
        self.positions = []
        self.sensor_readings = []
        self.times = []
        name = "poses"
        self.filepath = os.path.join(dir_path, "..", "logs", name + ".txt")
        f = open(self.filepath, "a")
        f.write("# Time, Sensor_reading, x, y, z" + "\n")
        f.close()

    def write_file(self):
        # only converter from list of list to comma seperated and new line
        f_string = '\n'.join(["{}, {}, {}".format(time, self.sensor_readings[i], ", ".join(
            str(pos) for pos in self.positions[i])) for i, time in enumerate(self.times)])

        f = open(self.filepath, "a")
        f.write(f_string + "\n")
        f.close()

    def clean_up(self):
        self.positions = []
        self.sensor_readings = []
        self.times = []

    def add_data(self, secs, nsecs, pos, sensor_reading):
        # print(pos)
        # np.append(self.positions.xyzs, pos)
        # np.append(self.positions.times, time)
        time = float(secs + float(nsecs)*1e-9)
        self.positions.append([pos.x, pos.y, pos.z])
        self.sensor_readings.append(sensor_reading)
        self.times.append(time)
        if len(self.times) % buffer_size == 0:
            self.write_file()
            self.clean_up()


class Sources():

    def __init__(self, pos_array):
        self._src_pos_array = np.array(pos_array)

    def inverse_euclidian_dist(self, dist_array):
        # calculate euclidian distances for each source
        eucli_dist = np.array([np.linalg.norm(dist) for dist in dist_array])
        print("euclidian distances: " + str(eucli_dist))
        # inverse it since it should be more present the closer it is
        inverse_eucli_dist_array = 1.0/eucli_dist
        print("inverse euclidian distances: " + str(inverse_eucli_dist_array))
        # sum all the inversed euclidian distances
        sum_inverse_dist = np.sum(inverse_eucli_dist_array)
        print("summed inverse distance: " + str(sum_inverse_dist))
        return sum_inverse_dist

    def light_intensity(self, dist_array):
        # calculate euclidian distances for each source
        eucli_dist = np.array([np.linalg.norm(dist) for dist in dist_array])
        print("euclidian distances: " + str(eucli_dist))
        # calculate intensity for each source
        intensity_array = 1.0/np.square(eucli_dist)
        print("intensities: " + str(intensity_array))
        # sum each intensity
        sum_intensity = np.sum(intensity_array)
        print("summed intensities: " + str(sum_intensity))
        return sum_intensity

    def get_relative_value(self, rob_pose):
        # does not take orientation into account
        # TODO take orientation into account
        print("Source positions: \n" + str(self._src_pos_array))
        rob_pos = np.array(
            [rob_pose.position.x, rob_pose.position.y, rob_pose.position.z])
        print("Robot Position: \n" + str(rob_pos))
        dist_array = np.array([abs(rob_pos - pos)
                              for pos in self._src_pos_array])
        print("distance to each source: \n" + str(dist_array))

        if method == Method.INV_EUC:
            value = self.inverse_euclidian_dist(dist_array)
        elif method == Method.INTENSITY:
            value = self.light_intensity(dist_array)
        else:
            value = self.inverse_euclidian_dist(dist_array)

        print("\n")

        return value


class Sub():
    def __init__(self, sources):
        self.sources = sources  # gain access to source class
        self.log = Log()
        self.simulation_flag = True
        self.listener()
        self.pose = None

        # initialize publisher
        self.pub = rospy.Publisher(
            'bluerov2/our_sensor', Illuminance, queue_size=10)

        # initialize sensor type
        self.sensor = Illuminance()

    def listener(self):
        # initialize subscriber
        # In simulations, the pose has a ground truth
        if (self.simulation_flag):
            rospy.Subscriber("/bluerov2/pose_gt", Odometry,
                             self.callback_simulation)
        else:
            #            rospy.Subscriber("/bluerov2/odometry/filtered",
            #                             Odometry, self.callback_simulation)
            rospy.Subscriber("/bluerov2/waterlinked/pose_with_cov_stamped",
                             PoseWithCovarianceStamped, self.callback_posstamp)

    def callback_simulation(self, odom):
        # Do this when something is published to intilized topic
        self.odom = odom
        self.pose = self.odom.pose.pose  # Extract what we need
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose.position)

    def callback_posstamp(self, posstamp):
        self.odom = posstamp
        self.pose = posstamp.pose.pose

    def publish_readings(self):
        # retrieve artifical value
        if self.pose == None:
            return

        self.sensor.illuminance = self.sources.get_relative_value(self.pose)
        secs = self.odom.header.stamp.secs
        nsecs = self.odom.header.stamp.nsecs
        self.log.add_data(secs, nsecs, self.pose.position,
                          self.sensor.illuminance)

        # publish the sensor msg
        self.pub.publish(self.sensor)


def main_loop():
    #sources = Sources([[10, 22, -1]])
    sources = Sources([[10, 15, -1], [10, 20, -1], [10, 22, -1],
                      [12.5, 20, -1], [12.5, 25, -1], [2, 40, -1]])
    print("Setup sub")
    sub = Sub(sources)
    print("Sources ready")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # sensor.illuminance += 0.1
        # rospy.loginfo(sensor.illuminance)
        sub.publish_readings()
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('node_name', anonymous=True)
    # init sources

    main_loop()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
