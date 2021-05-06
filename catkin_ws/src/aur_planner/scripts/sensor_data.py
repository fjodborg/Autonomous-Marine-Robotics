#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Illuminance
from geometry_msgs.msg import Pose
import numpy as np


class Sources():
    def __init__(self, pos_array):
        self._pos_array = np.array(pos_array)

    def inverse_euclidian_dist(self, dist_array):
        eucli_dist = np.array([np.linalg.norm(dist) for dist in dist_array])
        print("euclidian distances: " + str(eucli_dist))
        inverse_eucli_dist = 1.0/eucli_dist
        print("inverse euclidian distances: " + str(inverse_eucli_dist))
        sum_inverse_dist = np.sum(inverse_eucli_dist)
        print("summed inverse distance: " + str(sum_inverse_dist))

        return sum_inverse_dist

    def get_relative_value(self, rob_pose):
        # does not take orientation into account
        # TODO take orientation into account
        rob_pos = np.array([rob_pose.position.x, rob_pose.position.y, rob_pose.position.z])
        print("Source positions: \n" + str(self._pos_array))
        print("Robot Position: \n" + str(rob_pos))
        dist_array = np.array([abs(rob_pos - pos) for pos in self._pos_array])
        print("distance to each source: \n"+ str(dist_array))
        value = self.inverse_euclidian_dist(dist_array)
        print("\n")

        return value


class Sub():
    def __init__(self, sources):
        self.sources = sources
        self.listener()
        self.pose = None

    def listener(self):
        rospy.Subscriber("pose", Pose, self.callback)

    def callback(self, pose):
        self.pose = pose
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose.position)
        self.sources.get_relative_value(pose)


def main_loop():
    pub = rospy.Publisher('sensor', Illuminance, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    sensor = Illuminance()
    sensor.illuminance = 0 
    while not rospy.is_shutdown():
        # sensor.illuminance += 0.1
        # rospy.loginfo(sensor.illuminance)
        pub.publish(sensor)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('node_name', anonymous=True)
    sources = Sources([[0, 2, 3], [3, 4, 5]])
    sub = Sub(sources)
    main_loop()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()