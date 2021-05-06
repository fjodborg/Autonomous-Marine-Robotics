#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose, Point


def main_loop():
    rate = rospy.Rate(10)
    pos_array = []
    x_range = 20
    y_range = 20
    for x in range(x_range):
        for y in range(y_range):
            if x % 2:
                y = y_range - y - 1
            print(x, y)
            pos_array.append([x, y, 0])  # pos(ition)s
    pose = Pose()

    # initialize publisher
    pub = rospy.Publisher('pose', Pose, queue_size=10)

    for pos in pos_array:
        if rospy.is_shutdown():
            break

        pose.position = Point(*pos)
        print(pose)

        pub.publish(pose)

        rate.sleep()

    exit()


if __name__ == '__main__':
    rospy.init_node('node_name', anonymous=True)
    # init sources
    main_loop()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()