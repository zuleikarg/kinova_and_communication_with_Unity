#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Quaternion, PoseArray
def nodo_envia():
	pose=Pose()
	pub = rospy.Publisher('/pose_rot', Pose, queue_size=10)
	rospy.init_node('nodo_envia', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		
		
		pub.publish(pose)
		print(pose)
		rate.sleep()
if __name__ == '__main__':
	nodo_envia()
