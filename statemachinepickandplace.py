#!/usr/bin/env python2.7  
import time
import roslib
roslib.load_manifest('auto_labeller')
import rospy
import math
from math import radians
import tf
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import sys
import copy
import moveit_commander
import os, os.path
import moveit_msgs.msg
from random import random, randrange, randint
from datetime import date, datetime

#The box bounderies....
XMAX = 0.38
XMIN = 0.28
YMAX = 0.137
YMIN = -0.096
#Home position
HOMEX = 0.3091
HOMEY = 0.0048
HOMEZ = 0.4234

locerror = 0.01 # max error in movement
moveit_commander.roscpp_initialize(sys.argv)
# Instantiate CvBridge
bridge = CvBridge()

image_color = "/camera/color/image_raw"
folder_path = "/home/lab/Pictures/"
counter = 0
error = 0.05 # error from the camera, since it is tilted
suctionCup = 0.054 #decreas if want to go further down 0.034 length from endeffector to end of suction cup red suction 5.5cm
dropSpace = 0.015
# green 4.5cm
def endPos(msg,rot): 
    # Copying for simplicity
    position = msg.pose.position
    quat = msg.pose.orientation
    #Find difference between camera and suction end
    diff=PointStamped()
    diff.header.frame_id = str(msg.header.frame_id) #camera_color_optical_frame
    diff.header.stamp = rospy.Time(0)
    diff.point.x=position.x  #z
    diff.point.y=position.y #x
    diff.point.z=position.z #y 
    p=listener.transformPoint("panda_suction_end",diff)

    pickPoint=PointStamped()
    pickPoint.header.frame_id = "panda_suction_end"
    pickPoint.header.stamp = rospy.Time(0)
    pickPoint.point.x=p.point.x
    pickPoint.point.y=p.point.y
    pickPoint.point.z=p.point.z-suctionCup
    p = listener.transformPoint("panda_link0",pickPoint)
    
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = rot[0]
    pose_target.orientation.y = rot[1]
    pose_target.orientation.z = rot[2]
    pose_target.orientation.w = rot[3]
    pose_target.position.x = p.point.x
    pose_target.position.y = p.point.y
    pose_target.position.z = p.point.z
    if(pose_target.position.x> XMAX or pose_target.position.x < XMIN or pose_target.position.y> YMAX or pose_target.position.y < YMIN ):
        rospy.loginfo("Error position out of bounderies")
        return homePos()
    return pose_target     
    
def captureImage(state):
    global counter
    #path = folder_path+"/"+state
    msg = rospy.wait_for_message(image_color, Image)
    color_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    imgname = str(counter).zfill(4) + ".png" 
    cv2.imwrite(os.path.join(dirName, imgname), color_frame)
    counter = counter+1

def currPos(pos):
    
    pickPoint=PointStamped()
    pickPoint.header.frame_id = "panda_suction_end"
    p = listener.transformPoint("panda_link0",pickPoint)
    x,y,z = pos.pose.position.x, pos.pose.position.y, pos.pose.position.z
    while(not rospy.is_shutdown() and not(abs(x-p.point.x)<locerror and abs(y-p.point.y)<locerror and abs(z-p.point.z)<locerror)):
        p = listener.transformPoint("panda_link0",pickPoint)
        x,y,z = pos.pose.position.x, pos.pose.position.y, pos.pose.position.z
        continue
    print("Robot in right place")

def homePos():
    #global moveit_msgs
    group.set_named_target("ready")
    plan1 = group.plan()
    group.go(wait=True)

def rotate():
    group.clear_pose_targets()
    #value = float(random()*radians(70)) # possible to go from -166 to 166
    value = float(radians(randint(-30, 120))) # 45 is straight
    #print "Value", value
    group_variable_values = group.get_current_joint_values()
    #print "============ Joint values: ", group_variable_values
    group_variable_values[6] = value
    print(radians(value))
    group.set_joint_value_target(group_variable_values)

    plan1 = group.plan()
    group.go(wait=True)
    #print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(0.3)
    

def moveToOtherSide(pos, moveBottle):
    #pos.position.x = pos.position.x
    pos.position.y = -1*pos.position.y
    pos.position.z = pos.position.z+dropSpace
    if(moveBottle):
        moveAbove(pos,False)
    pos.position.z = pos.position.z-dropSpace
    return pos
    #pub.publish(pe)
    #currPos(pe)

def moveAbove(pose_target,drop):
    before = pose_target.position.z

    pose_target.position.z = 0.25
    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)
    if(drop==True):
        pose_target.position.z = before+dropSpace
        group.set_pose_target(pose_target)
        plan1 = group.plan()
        group.go(wait=True)
        pose_target.position.z = pose_target.position.z-dropSpace
    elif(drop==False):
        pose_target.position.z = before
        group.set_pose_target(pose_target)
        plan1 = group.plan()
        group.go(wait=True)

def stateMachine(currState):
    global endPosition
    if currState == 1:
        captureImage("")
        value =0
        while value != 2: 
            value = input("Enter 2 to get coordinates:\n")
            value = int(value)
        stateMachine(value)
    elif currState == 2:
        currPoint=PointStamped()
        currPoint.header.frame_id = "panda_suction_end"
        home = listener.transformPoint("panda_link0",currPoint)
        if ((abs(HOMEX-home.point.x)<locerror and abs(HOMEY-home.point.y)<locerror and abs(HOMEZ-home.point.z)<locerror)):
            try:
                (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
                msg = rospy.wait_for_message("/pandaposition", PoseStamped)
                rospy.loginfo("Received at goal message!")
                endPosition = endPos(msg,rot) # transforming
                #print(endPosition)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("look up fail")    
            rospy.loginfo(endPosition)
            value = input("Enter 3 to move robot to coordinates, 0 to get new:\n")
            value = int(value)
            if (value == 3):
                stateMachine(value)
            else:
                stateMachine(2)
        else:
            print("robot not in home pos")  
    elif currState == 3:
        start = time.time()
        captureImage("")
        moveAbove(endPosition, False)
        pubSuc.publish(Bool(True))
        rospy.sleep(0.2)
        #homePos()
        lastPos = moveToOtherSide(endPosition,True)
        pubSuc.publish(Bool(False))
        rospy.sleep(0.1)
        forValue = 299
        for i in range(forValue):
            try:
                homePos() # Take photo in home pos
                captureImage("")
                moveAbove(lastPos, False)
                pubSuc.publish(Bool(True))
                #homePos() # Take photo in home pos
                lastPos = moveToOtherSide(lastPos, False)
                moveAbove(lastPos, True)
                rotate()
                pubSuc.publish(Bool(False))
                print("curr time",(time.time()-start))
                if i == forValue-1:
                    homePos()
                    captureImage("")
            except KeyboardInterrupt:
                print('Interrupted')
                break
        
        end = time.time()
        rospy.loginfo("TIME:")
        rospy.loginfo(end - start)
        rospy.loginfo("iterations finished:", forValue+1)
        stateMachine(1)
        value = input("Enter 1 to create new dir and 2 to add to curr dir:\n")
        value = int(value)
        if value == 1:
            stateMachine(1)
        elif value == 2:
            stateMachine(2)
"""     elif currState == 4:

    elif currState == 5:

    elif currState == 6: """

if __name__ == '__main__':
    #rostopic pub  std_msgs/Bool true --once
    pubSuc = rospy.Publisher('/vacuum/set_suction_on', Bool, queue_size=1) # Talk to the festo ovem suction cup
    rospy.init_node('pick_and_place')
    listener = tf.TransformListener()
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("panda_arm")
    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10) # Talks to the moveit motion planner
    homePos()
    while not rospy.is_shutdown():
        empty = input("Take image of an empty bin. Enter 1 to capture it\n")
        empty = int(empty)
        now = datetime.now()
        # dd/mm/YY H:M:S
        dt_string = now.strftime("%d-%m-%Y-%H:%M")
        # Create director
        dirName = folder_path + "/" + str(dt_string)  
        try:
            # Create target Directory
            os.mkdir(dirName)
            print("Directory " , dirName ,  " Created ") 
            counter = 0
        except:
            print("Directory " , dirName ,  " already exists") 
        stateMachine(empty)


