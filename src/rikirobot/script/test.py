#! /usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from darknet_ros_msgs.msg import BoundingBoxes

class GoToPose():
    def __init__(self):
        self.goal_sent = False 

	# What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)

        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 120 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(120)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

def callback(msg):    
    print (msg.data)   
    words = '主人我找到了一个' 
    if msg.data == "卧室":
        print ("OK")
        try:
            navigator = GoToPose()
            position = {'x': 2.75, 'y' : 5.23}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])

            success = navigator.goto(position, quaternion)

            if success:
                #pub_loc = rospy.Publisher("voiceWords",String,queue_size=1)
                strs="我已经到卧室啦，主人"  
                pub_loc.publish(strs)

                rospy.sleep(5)
                navigator = GoToPose()
                position = {'x': 2.75, 'y' : 5.23}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.707, 'r4' : 0.707}
                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                rospy.sleep(5)

                navigator = GoToPose()
                position = {'x': 2.75, 'y' : 5.23}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}
                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                rospy.sleep(5)

                senten = words+something_class
                #pub_loc.publish(something_class)
                pub_loc.publish(senten)
                

                navigator = GoToPose()
                position = {'x': 2.75, 'y' : 5.23}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.707, 'r4' : -0.707}
                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                rospy.sleep(5)
            else:
                rospy.loginfo("The base failed to reach the desired pose")

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)
        except rospy.ROSInterruptException:
            rospy.loginfo("Ctrl-C caught. Quitting")

    if msg.data == "客厅":
        print ("OK")
        try:
            navigator = GoToPose()
            position = {'x': 6.11, 'y' : 5.32}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])

            success = navigator.goto(position, quaternion)

            if success:
                #pub_loc = rospy.Publisher("voiceWords",String,queue_size=1)
                strs="我已经到客厅啦，主人"  
                pub_loc.publish(strs)

                rospy.sleep(5)
                navigator = GoToPose()
                position = {'x': 6.11, 'y' : 5.32}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.707, 'r4' : 0.707}
                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                rospy.sleep(5)

                navigator = GoToPose()
                position = {'x': 6.11, 'y' : 5.32}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}
                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                rospy.sleep(5)

                senten = words+something_class
                
                #pub_loc.publish(senten)

                pub_loc.publish(something_class)
                
                navigator = GoToPose()
                position = {'x': 6.11, 'y' : 5.32}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.707, 'r4' : -0.707}
                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                rospy.sleep(5)
            else:
                rospy.loginfo("The base failed to reach the desired pose")

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)
        except rospy.ROSInterruptException:
            rospy.loginfo("Ctrl-C caught. Quitting")
def callback2(msg):
    global  something_class 
    something_class = msg.bounding_boxes[0].Class

rospy.init_node('subscr')
 
subcmd = rospy.Subscriber('goto_command', String ,callback, queue_size=5) 
sub1 = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback2,queue_size=5)
pub_loc = rospy.Publisher("voiceWords",String,queue_size=2)
rospy.spin()

