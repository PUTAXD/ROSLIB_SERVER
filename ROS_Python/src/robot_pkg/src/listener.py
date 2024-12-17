import rospy
from robot_pkg.msg import server2pc

def callback(data):
    rospy.loginfo(f"Received message: {data.say}")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.loginfo("Listening to server2pc")
    rospy.Subscriber("/server2pc", server2pc, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()