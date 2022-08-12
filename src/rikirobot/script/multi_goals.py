#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2017 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import string
import math
import time
import sys

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped



class Goals:
    def __init__(self, retry, map_frame):
        self.sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.statusCB, queue_size=10)
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)   
       
        # params & variables
     	self.goalX = 0
        self.goalY = 0
        self.retry = retry
        self.goalMsg = PoseStamped()
        self.goalMsg.header.frame_id = map_frame
        self.goalMsg.pose.orientation.z = 0.0
        self.goalMsg.pose.orientation.w = 1.0
        # Publish the first goal
        time.sleep(1)
        self.goalMsg.header.stamp = rospy.Time.now()
        self.goalMsg.pose.position.x = self.goalX
        self.goalMsg.pose.position.y = self.goalY
        self.pub.publish(self.goalMsg) 
        print("a")

    def statusCB(self, data):
        if data.status.status == 3: # reached
            pub_loc = rospy.Publisher("voiceWords",String,queue_size=1)
            strs="我已经到目标啦，主人"  
            pub_loc.publish(strs)

def callback(msg):
    global goalX, goalY
    if msg.data =="客厅":
        goalX = 6.11
        goalY = 5.32
    elif msg.data =="书房":
        goalX = 8.1
        goalY = 2.07
    elif msg.data =="卧室":
        goalX = 2.75
        goalY = 5.23
        
    elif msg.data =="走廊":
        goalX = 3.43
        goalY = 1.75


                


if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('voice_goals', anonymous=True)
        subcmd = rospy.Subscriber('goto_command',String ,callback, queue_size=1)  
        retry = rospy.get_param('~retry', '1') 
	map_frame = 'map' 

        global goalX, goalY
        
        mg = Goals(retry, map_frame)          
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")



