#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Illuminance
from geometry_msgs.msg import Pose


def talker():
    pub = rospy.Publisher('sensor', Illuminance, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    sensor = Illuminance()
    sensor.illuminance = 0 
    while not rospy.is_shutdown():
        sensor.illuminance += 0.1
        rospy.loginfo(sensor.illuminance)
        pub.publish(sensor)
        rate.sleep()


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber("pose", Pose, callback)


def callback(pose):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose.position)


if __name__ == '__main__':
    rospy.init_node('node_name', anonymous=True)
    listener()
    talker()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()