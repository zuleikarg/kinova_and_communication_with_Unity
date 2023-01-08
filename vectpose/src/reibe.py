#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Pose, Quaternion, PoseArray

from unity_robotics_demo_msgs.msg import PosRot
vect=PoseArray()
tam=0
pub = rospy.Publisher('/Poses', PoseArray, queue_size=1000) 
def callback(data):
	global tam
	if(data.pos_y == 100.0):
		print(data)
		tam=data.pos_x
	else:
		pose=Pose()
		pose.position.x=data.pos_x
		pose.position.y=data.pos_y
		pose.position.z=data.pos_z
		pose.orientation.x=data.rot_x
		pose.orientation.y=data.rot_y
		pose.orientation.z=data.rot_z
		pose.orientation.w=data.rot_w
		vect.poses.append(pose)
		#print(data.pos_x)
		#print(vect.poses[0].pos_y)
		if(len(vect.poses)==tam):
			pub.publish(vect)
			print(vect)
			tam=0
			vect.poses.clear()
			print("borrado")
			
			
			
			
	
	
	
	
def nodo_recibe():
	
	rospy.init_node('nodo_recibe', anonymous=True)
	
	rospy.Subscriber("pos_rot", PosRot, callback)
	
	rospy.spin()

if __name__ == '__main__':
	
	nodo_recibe()
