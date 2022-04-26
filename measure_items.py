#!/usr/bin/env python3  
import time
from typing import Container
import roslib
#roslib.load_manifest('auto_labeller')
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
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import sys
import numpy as np
import copy
import moveit_commander
import os, os.path
import moveit_msgs.msg
from random import random, randrange, randint
from datetime import date, datetime
#The box bounderies....
XMAX = 0.36 # 0.38
XMAX2 = 0.405
XMIN = 0.28
YMAX = 0.137
YMIN = -0.096
#Home position
HOMEX = 0.3345#0.3091
HOMEY =  0#0.0048
HOMEZ = 0.2346#0.4234

locerror = 0.01 # max error in movement
#moveit_commander.roscpp_initialize(sys.argv)
# Instantiate CvBridge
bridge = CvBridge()

image_color = "/camera/color/image_raw"
image_depth = "/camera/aligned_depth_to_color/image_raw"
folder_path = "/home/lab/Pictures/data"
counter = 0
error = 0.05 # error from the camera, since it is tilted
# green 4.5cm

# Adds a marker at the provided coordinates which can be visualized in rviz 
def addMarker(pickp,refframe,color=True):

    marker = Marker()
    marker.header.frame_id = refframe
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    if(color == True):
        marker.color.r = 1.0
    else:
        marker.color.g = 1.0

    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = pickp.position.x
    marker.pose.position.y = pickp.position.y
    marker.pose.position.z = pickp.position.z
    marker.lifetime = rospy.Duration(0)

    markpub.publish(marker)

# Moves down vertically towards a pick point, takes in a force goal as a stopping condition indicating the pick point has been reached
def movetotouch(initial_pose, force_goal):
    totdistance = 0
    # Vectors to collect force/distance data
    force = []
    distance = []
    xvec = []
    yvec = []
    waypoints = []
    realdist = []

    #Setting initial values of force and distance vectors
    forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
    initpos = initial_pose.position.z
    initforce = forcemsg.wrench.force.z
    increment = 0.001
    forceval=0

    while(totdistance < 0.05 and forceval < force_goal):
        initial_pose.position.z -= increment
        cartesian_traject(initial_pose,0.0005,0.5,True)
        
        #input("press to increment down")

        forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
        forceval=float(abs(forcemsg.wrench.force.z - initforce))
        force.append(forceval)
        totdistance += increment 
        distance.append(totdistance)
        (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
        xvec.append(trans[0])
        yvec.append(trans[1])
        realdist.append(initpos-initial_pose.position.z)
        print(f'Force: {forceval} - Distance: {totdistance}')
        print(f'{forcemsg.wrench.force.z}')
        input("Press to continue")

def moveAbove(pose_target, camdepth):
    captureImage(currbox)
    before = pose_target.position.z
    print("Move above!")
    # Move the suction cup slightly above the object (panda_suction_end 10 cm from target)
    pose_target.position.z = before+0.01 #0.12
    print(pose_target)
    # group.set_pose_target(pose_target)
    # plan1 = group.plan()
    # group.go(wait=True)
    # group.clear_pose_targets()
    cartesian_traject(pose_target,0.05,0.5)
    print("Hovering above")
    currentp=group.get_current_pose()
    currentp=pose_unstamped(currentp)
    # Moves vertically towards target pick point with a stopping condition of 2 N vertical force on the end effector
    movetotouch(currentp,0.5)
    # errorx=pose_target.position.x - currentp.pose.position.x
    # errory=pose_target.position.y - currentp.pose.position.y
    # errorz=pose_target.position.z - currentp.pose.position.z
    # print(f'errors: x: {errorx} - y={errory} - z={errorz}')

    input("Press any key to continue")
    #downspace = 0 
    #pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
    # pubSuc.publish(Bool(True))
    # while(pressure.data > -0.1):
    #     if pose_target.position.z < 0.16 or downspace > 0.04:
    #         homePos()
    #         rospy.sleep(0.3)
    #         pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
    #         if pressure.data > -0.1:
    #             pubSuc.publish(Bool(False))
    #             return 0
    #     else:        
    #         pose_target.position.z = before-downspace
    #         print(f'z target position at: {pose_target.position.z} m')
    #         print(f'Pressure value at: {pressure}')
    #         group.set_pose_target(pose_target)
    #         plan1 = group.plan()
    #         group.go(wait=True)
    #         group.clear_pose_targets()
    #         downspace = downspace + 0.0015
    #         input("Push any button to increment down")
    #     pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
        
    # print("outside loop")    
    # pose_target.position.z = before + 0.10
    # group.set_pose_target(pose_target)
    # plan1 = group.plan()
    # group.go(wait=True)
    # group.clear_pose_targets()

    # pose_target.position.z = before+downspace
    return 1

def moveToOtherBox(currpos): #move the item that is picked to a "locked" location...
    homePos()
    currpos.position.x = 0.45
    currpos.position.y = -0.42
    currpos.position.z = 0.43
    currpos.orientation.x = 0.99999
    currpos.orientation.y = 0.006
    currpos.orientation.z = 0.008
    currpos.orientation.w = 0.007
    group.set_pose_target(currpos)
    plan1 = group.plan()
    group.go(wait=True)
    pubSuc.publish(Bool(False))

# Takes in a geometry_msgs.msg.Pose() and uses compute_cartesian_path to get a certesian trajectory and subsequently executes it
def cartesian_traject(waypoint,step,vel_scale,pause=False):
    
    group.clear_pose_targets()
    #print(waypoint.position.z)
    waypoints = []
    waypoints.append(copy.deepcopy(waypoint))  
    (plan, fraction) = group.compute_cartesian_path(
                                                          waypoints,   # waypoints to follow
                                                          step,        # eef_step
                                                          0         # jump_threshold
    )

    plan = group.retime_trajectory(robot.get_current_state(),plan,vel_scale)
    group.execute(plan)

    if pause:
        rospy.sleep(0.2)

def pose_unstamped(pose_target_stamped):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = pose_target_stamped.pose.orientation.x
    pose_target.orientation.y = pose_target_stamped.pose.orientation.y
    pose_target.orientation.z = pose_target_stamped.pose.orientation.z
    pose_target.orientation.w = pose_target_stamped.pose.orientation.w
    pose_target.position.x = pose_target_stamped.pose.position.x
    pose_target.position.y = pose_target_stamped.pose.position.y
    pose_target.position.z = pose_target_stamped.pose.position.z

    return pose_target

def endPos(msg,rot): 
    # Copying for simplicity
    position = msg.pose.position
    quat = msg.pose.orientation
    # The pickpoint relative to the camera
    camframePt=PointStamped()
    camframePt.header.frame_id = str(msg.header.frame_id) #camera_color_frame
    camframePt.header.stamp = rospy.Time(0)
    camframePt.point.x=position.x #z
    camframePt.point.y=position.y #x
    camframePt.point.z=position.z #y 
    # The pickpoint relative to the suction end
    #p=listener.transformPoint("panda_suction_end",diff)

    #pickPoint=PointStamped()
    #pickPoint.header.frame_id = "panda_suction_end"
    #pickPoint.header.stamp = rospy.Time()
    #pickPoint.point.x=p.point.x
    #pickPoint.point.y=p.point.y
    #pickPoint.point.z=p.point.z-suctionCup
    p = listener.transformPoint("panda_link0",camframePt)   
    
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = rot[0]
    pose_target.orientation.y = rot[1]
    pose_target.orientation.z = rot[2]
    pose_target.orientation.w = rot[3]
    # pose_target.orientation.x = 1
    # pose_target.orientation.y = 0
    # pose_target.orientation.z = 0
    # pose_target.orientation.w = 0
    pose_target.position.x = p.point.x
    pose_target.position.y = p.point.y
    pose_target.position.z = p.point.z
    currentbox = [quat.x, quat.y, quat.z, quat.w]
    #addMarker(pose_target,"panda_link0", False)
    #print(currentbox)
    return pose_target, quat, camframePt.point.x    
    
def captureImage(currentbox):
    global counter
    colormsg = rospy.wait_for_message(image_color, Image)
    depthmsg = rospy.wait_for_message(image_depth, Image)
    color_frame = bridge.imgmsg_to_cv2(colormsg, "bgr8")
    depth_frame = bridge.imgmsg_to_cv2(depthmsg, desired_encoding="passthrough")
    depth_image = np.array(depth_frame, dtype=np.float32)
    depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image), cv2.COLORMAP_RAINBOW)
    imgname = str(counter).zfill(4)
    f= open(os.path.join(dirName, imgname+".txt"),"w+")
    if currentbox != 0:
        f.write("0 %.9f %.9f %.9f %.9f\r\n" % (float(currentbox.x),float(currentbox.y),float(currentbox.z),float(currentbox.w)))
    f.close() 
    cv2.imwrite(os.path.join(dirName, imgname+"color.png"), color_frame)
    cv2.imwrite(os.path.join(dirName, imgname+"depth.png"), depth_frame)
    counter = counter+1

def homePos():
    print("Moving to home position")
    group.clear_pose_targets()
    #group_variable_values = group.get_current_joint_values()
    #group_variable_values[0] = float(radians(0))
    #group_variable_values[1] = float(radians(-25))
    #group_variable_values[2] = float(radians(1))
    #group_variable_values[3] = float(radians(-133))
    #group_variable_values[4] = float(radians(0))
    #group_variable_values[5] = float(radians(109))
    #group_variable_values[6] = float(radians(45)) # 45 is straight
    #group.set_joint_value_target(group_variable_values)

    # Joint positions in radians (L515)
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = 0.039827141041538304
    group_variable_values[1] = -0.6856789446295354
    group_variable_values[2] = -0.051094866318138026
    group_variable_values[3] = -2.6372590758340397
    group_variable_values[4] = -0.022729468238022594
    group_variable_values[5] = 1.9992456805838477
    group_variable_values[6] = 0.7680859528355461 # 45 is straight
    group.set_joint_value_target(group_variable_values)

    # Joint positions in radians (D435)
    # group_variable_values = group.get_current_joint_values()
    # group_variable_values[0] = 0.020526881909422708
    # group_variable_values[1] = -0.5317800948996293
    # group_variable_values[2] = -0.1270579353005081
    # group_variable_values[3] = -2.6110645072334693
    # group_variable_values[4] = -0.018342747257815466
    # group_variable_values[5] = 2.139517307692104
    # group_variable_values[6] = 0.6733780175112191
    # group.set_joint_value_target(group_variable_values)
    
    plan1 = group.plan()
    group.go(wait=True)
    group.clear_pose_targets()
 

def stateMachine(currState):
    global endPosition, start, currbox, dirName, camdepth
    if currState == 0: #The first state starts by creating a new directory to save the images.
        folder = ""
        print("Create new dir")
        #folder = input("Folder name\n") # user creates a folder
        folder="eidur"
        dirName = folder_path + "/" + folder
        try:
            # Create target Directory
            os.mkdir(dirName)
            print("Directory " , dirName ,  " Created ") 
            counter = 0
        except:
            print("Directory " , dirName ,  " already exists") 
        return 1
    elif currState == 1: # The second state waits until the user wants coordinates by entering the desired number. 
        value = 2
        while value != 2: 
            value = input("Enter 2 to get coordinates:\n")
            value = int(value)
        return value
    elif currState == 2: 
        # The third state waits for a message from the find\_pickpoint.py through a ROS topic where it gets an object position from the camera, 
        # and then the TF listener transforms the object position in camera coordinates to world coordinates.
        print("State 2") 
        currPoint=PointStamped()
        currPoint.header.frame_id = "panda_suction_end"
        home = listener.transformPoint("panda_link0",currPoint)
        #home = group.get_current_pose()
        # Checking if the end effector is in the right initial position
        if ((abs(HOMEX-home.point.x)<locerror and abs(HOMEY-home.point.y)<locerror and abs(HOMEZ-home.point.z)<locerror)):
            try:
                # Getting current position and rotation of current position of panda_suction_end
                (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time())
                # Setting end effector parallel to the table
                q = quaternion_from_euler(math.pi, 0, 0)
                msg = rospy.wait_for_message("/pandaposition", PoseStamped)
                rospy.loginfo("Received at goal message!")
                addMarker(msg.pose, "camera_color_frame")
                input("Review position of marker")
                if(msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0 ):
                    return 6
                endPosition, currbox, camdepth = endPos(msg,q) # transforming from camera frame to world frame
                rospy.loginfo(endPosition)
                #input("Inspect final position before continuing")
                if endPosition == "None":
                    
                    return 2
                else: 
                    emptycount = 0
                    return 3
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("look up fail")    
        else:
            homePos()
            return 2

    elif currState == 3:
        # The fourth state starts by taking an image of the bin before moving above the object, 
        # it then turns on the suction and slowly goes down to the object.  
        #The fourth state finishes when the suction pressure increases and that means the suction cup has picked up the object 
        # and then moves to a fixed home position.
        start = time.time()
        pick=moveAbove(endPosition,camdepth)
        if pick == 0:
            return 2
        rospy.sleep(0.2)
        #homePos()
        return 4
        
    elif currState == 4:
        # The fifth state moves the object above another bin, drops it, and ends by going to a fixed home position. 
        # When the fifth state has finished and there are items in the bin it goes back to the third state, 
        # but if the bin is empty it goes to the sixth state.
        #moveToOtherBox(endPosition)
        homePos()
        rospy.sleep(2)
        end = time.time()
        rospy.loginfo("TIME:")
        rospy.loginfo(end - start)
        return 2
    elif currState == 6:
        # The sixth state takes an image of an empty bin and waits until the user wants to start
        # again with new items in the bin and then goes to the first state again. 
        exitstate = -1
        homePos()
        pubSuc.publish(Bool(False))
        #captureImage(0)
        while exitstate != 0: 
            exitstate = input("Enter 0 to begin again and create new directory:\n")
            exitstate = int(exitstate)
        print(exitstate)
        return 0
        
        
if __name__ == '__main__':
    emptycount = 0
    #rostopic pub  std_msgs/Bool true --once
    pubSuc = rospy.Publisher('/vacuum/set_suction_on', Bool, queue_size=1) # Talk to the festo ovem suction cup
    rospy.init_node('pick_and_place')
    listener = tf.TransformListener()
    markpub = rospy.Publisher("posmarker", Marker, queue_size=1)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("panda_arm")
    group.allow_replanning(True)
    group.set_planning_time(1)
    ref=group.get_pose_reference_frame()
    eef_link=group.get_end_effector_link()
    orient = group.get_goal_orientation_tolerance()
    goal_tol=group.get_goal_tolerance()
    print(f'Reference frame: {ref}')
    print(f'End effector link: {eef_link}')
    print(f'Goal tolerance: {goal_tol}')
    print(f'Orientation tolerance: {orient}')
    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10) # Talks to the moveit motion planner
    homePos()
    #empty = input("Enter 0 to start the statemachine\n")
    empty = 0
    now = datetime.now()
    # dd/mm/YY H:M:S
    dt_string = now.strftime("%d-%m-%Y-%H:%M")
    if empty == 0:
        currstate = stateMachine(empty)

    while not rospy.is_shutdown():
        currstate = stateMachine(currstate)
        
            

