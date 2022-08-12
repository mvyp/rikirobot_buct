#!/usr/bin/env python  
import rospy
from darknet_ros_msgs.msg import BoundingBoxes

def callback(data):
    print data.bounding_boxes[0].Class

def listener():
    rospy.init_node('topic_subscriber')

    sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
    if (data.bounding_boxes[0].Class==bottle):
        
    rospy.spin()

if __name__ == '__main__':
    listener()

