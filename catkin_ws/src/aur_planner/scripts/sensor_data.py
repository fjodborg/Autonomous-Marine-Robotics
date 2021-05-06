#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Illuminance
from geometry_msgs.msg import Pose
import numpy as np
from enum import Enum


class Method(Enum):
    INV_EUC = 1
    INTENSITY = 2


method = Method.INTENSITY


class Sources():
    def __init__(self, pos_array):
        self._pos_array = np.array(pos_array)

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
        print("Source positions: \n" + str(self._pos_array))
        rob_pos = np.array([rob_pose.position.x, rob_pose.position.y, rob_pose.position.z])
        print("Robot Position: \n" + str(rob_pos))
        dist_array = np.array([abs(rob_pos - pos) for pos in self._pos_array])
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
        self.listener()
        self.pose = None

        # initialize publisher
        self.pub = rospy.Publisher('sensor', Illuminance, queue_size=10)

        # initialize sensor type
        self.sensor = Illuminance()

    def listener(self):
        # initialize subscriber
        rospy.Subscriber("pose", Pose, self.callback)

    def callback(self, pose):
        # Do this when something is published to intilized topic
        self.pose = pose
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose.position)
        # retrieve artifical value
        self.sensor.illuminance = self.sources.get_relative_value(pose)
        # publish the sensor msg
        self.pub.publish(self.sensor)


def main_loop():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # sensor.illuminance += 0.1
        # rospy.loginfo(sensor.illuminance)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('node_name', anonymous=True)
    # init sources
    sources = Sources([[0, 2, 3], [3, 4, 5]])
    sub = Sub(sources)
    main_loop()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()