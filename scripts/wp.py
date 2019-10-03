#!/usr/bin/env python
import sys
import rospy
import tf
import math
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped, Point


cmd = TwistStamped()
orientation = 0.0
pos = Point()
fl = False
FB=0.0
LR=0.0

err_prev = 0
err_ang = 100
i_err = 0

kp_ang = 0.01
ki_ang = 0.0000001
kd_ang = 0.00003
tha = 0.6

def getError(goal):
	global err_ang, orientation
	err_ang = goal-orientation
	err_ang = math.atan2(math.sin(err_ang),math.cos(err_ang))*180/math.pi
	
	


def yawControl(goal):
	controlOrientation = False
	global cmd, orientation, err_prev, err_ang, i_err
	global kp_ang, ki_ang, kd_ang, tha

	getError(goal)

	d_err = err_ang-err_prev
	err_prev = err_ang
	i_err += err_ang

	ua = kp_ang*err_ang + kd_ang*d_err + ki_ang*i_err
	print('ua: ')
	print(ua)

	if ua>=tha:
		ua = tha
	elif ua <= -tha:
		ua = -tha

	cmd.twist.angular.z = ua


def pose(data):
	global orientation, fl, pos

	pos.x = data.pose.position.x
	pos.y = data.pose.position.y
	pos.z = data.pose.position.z

	qw = data.pose.orientation.w
	qx = data.pose.orientation.x
	qy = data.pose.orientation.y
	qz = data.pose.orientation.z
	(r,p,y) = tf.transformations.euler_from_quaternion([qx,qy,qz,qw])
	orientation = math.atan2(math.sin(y-math.pi/2),math.cos(y-math.pi/2))

	fl = True




def wp(goal):
	global err_ang,fl
	err_angTH = 5.0 
	
	rospy.Subscriber("/mavros/local_position/pose",PoseStamped,pose)
	pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size = 1)
	rospy.init_node('control', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	while not fl:
		pass
	getError(goal)
	print(goal)
	goal = math.atan2(math.sin(goal+orientation),math.cos(goal+orientation))
	print(goal)
	getError(goal)
	out = True
	i=0

	while not rospy.is_shutdown() and out:
		rate.sleep()
		if fl is True:
			getError(goal)
			if (err_ang< err_angTH) and (err_ang> -err_angTH):
				#cmd.twist.angular.z = 0
				i=i+1

			else:
				yawControl(goal)

			if i is 10:
				file1 = open("myfile.txt","a")
				line = str(err_ang)+"\n"
				file1.write(line)
				file1.close()
				out = False

			pub.publish(cmd)


if __name__ == '__main__':


	try:
		if len(sys.argv) is 2:
			goal = float(sys.argv[1])
			wp(goal)
		else:
			pass
	except rospy.ROSInterruptException:
		pass