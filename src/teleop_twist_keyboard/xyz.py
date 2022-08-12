#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
twist=None
msg = """
---------------------------
Moving around:
"左前"         "向前"          "右前"
"向左转"     "停止"          "向右转"
"左后"			"向后"			"右后"
anything else : stop


CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),	
		
	}

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }
def keys_cb(msg,twist_pub):
	global twist,speed,turn,x,th,status
	print (msg)
	print (vels(speed,turn))
	key=msg.data
	if key in moveBindings.keys():
		x = moveBindings[key][0]
		y = moveBindings[key][1]
		z = moveBindings[key][2]
		th = moveBindings[key][3]
	elif key in speedBindings.keys():
		speed = speed * speedBindings[key][0]
		turn = turn * speedBindings[key][1]

		print (vels(speed,turn))
		if (status == 14):
			print (msg)
		status = (status + 1) % 15
	else:
		x = 0
		y = 0
		z = 0
		th = 0
	
	twist.linear.x = x*speed
	twist.linear.y = y*speed
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = th*turn
	twist_pub.publish(twist)


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0
	turn=0
	speed = rospy.get_param("~speed", 0.2)
	turn = rospy.get_param("~turn", 0.3)
	
	rospy.init_node('voice_teleop')
	twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	rospy.Subscriber('keyCmd',String,keys_cb,twist_pub)
	rate=rospy.Rate(1)
	twist=Twist()
	while not rospy.is_shutdown():	  
		twist_pub.publish(twist)
        rate.sleep()
