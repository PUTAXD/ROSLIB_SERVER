#!/usr/bin/env python
# license removed for brevity
import rospy
from robot_pkg.msg import data_robot
from robot_pkg.msg import bs2pc_roslib
from robot_pkg.msg import pc2bs_roslib

def talker():
    robotPublisher = rospy.Publisher('/bs2pc_server', bs2pc_roslib, queue_size=10)
    bsPublisher = rospy.Publisher('/pc2bs_server', pc2bs_roslib, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    #     data_robot1 = data_robot()
    #     data_robot1.epoch = rospy.Time(rospy.get_time())
    #     data_robot1.pose_x = 111
    #     data_robot1.pose_y = 111
    #     data_robot1.theta = 111

    #     robotPublisher.publish(data_robot1)
    #     rospy.loginfo(data_robot1)
    #     robotPublisher.publish(data_robot1)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass