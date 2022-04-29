#!/usr/bin/env python3  
import time
from typing import Container
import roslib
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
from tf.transformations import quaternion_from_euler
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
import copy
import moveit_commander
import os, os.path
import moveit_msgs.msg
from random import random, randrange, randint
from datetime import date, datetime
from geometry_msgs.msg import WrenchStamped
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

# Mininum and maximum points of the box in world coordinates - Given that the bin hasn't been moved

YMIN = -0.1139
YMAX = 0.1838
XMIN = 0.3341
XMAX = 0.44374

moveit_commander.roscpp_initialize(sys.argv)
# Instantiate CvBridge
bridge = CvBridge()

image_color = "/camera/color/image_raw"
# folder_path = "/mnt/bigdata/eidur14/data"
folder_path = "/home/lab/pictures/data"
counter = 0

## Helper functions ##

# Adds a pose marker which can be viewed in Rviz
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

# Perform a cartesian trajectory using moveitcommander function compute_cartesian_path.
def cartesian_traject(waypoint,step,vel_scale,pause=False,sleeptime=0.6):
    
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
        rospy.sleep(sleeptime)

def pose_unstamped(pose_target_stamped):
    pose_targetFR = geometry_msgs.msg.Pose()
    pose_targetFR.orientation.x = pose_target_stamped.pose.orientation.x
    pose_targetFR.orientation.y = pose_target_stamped.pose.orientation.y
    pose_targetFR.orientation.z = pose_target_stamped.pose.orientation.z
    pose_targetFR.orientation.w = pose_target_stamped.pose.orientation.w
    pose_targetFR.position.x = pose_target_stamped.pose.position.x
    pose_targetFR.position.y = pose_target_stamped.pose.position.y
    pose_targetFR.position.z = pose_target_stamped.pose.position.z

    return pose_targetFR

def homePos():
    print("Moving to home position")
    group.clear_pose_targets()
    group.set_start_state_to_current_state()

    ## Different configurations for inital positions 
    #group_variable_values = group.get_current_joint_values()
    #group_variable_values[0] = float(radians(0))
    #group_variable_values[1] = float(radians(-25))
    #group_variable_values[2] = float(radians(1))
    #group_variable_values[3] = float(radians(-133))
    #group_variable_values[4] = float(radians(0))
    #group_variable_values[5] = float(radians(109))
    #group_variable_values[6] = float(radians(45)) # 45 is straight
    #group.set_joint_value_target(group_variable_values)

    # group_variable_values = group.get_current_joint_values()
    # group_variable_values[0] = 0
    # group_variable_values[1] = -0.6999209085682471
    # group_variable_values[2] = 0.015764035018303778
    # group_variable_values[3] = -2.479506837022749
    # group_variable_values[4] = -0.03019742977288034
    # group_variable_values[5] = 1.8284102745983333
    # group_variable_values[6] = 0.7890444130053123
    # group.set_joint_value_target(group_variable_values)

    # Joint positions in radians (D435)
    # group_variable_values = group.get_current_joint_values()
    # group_variable_values[0] = 0.07320734092148395
    # group_variable_values[1] = -0.6008668125573948
    # group_variable_values[2] = -0.16961717653901953
    # group_variable_values[3] = -2.6558488347211138
    # group_variable_values[4] = -0.07008392171724813
    # group_variable_values[5] = 2.1059772145880604
    # group_variable_values[6] = 0.7096239592979352
    # group.set_joint_value_target(group_variable_values)

    # Joint positions in radians (D435) - Level to bin plane - Final
    # Around 43 cm from bin bottom and with the camera aligned in the middle
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = 0.2405372671024597
    group_variable_values[1] = -0.7981669106111364
    group_variable_values[2] = -0.2941279667553149
    group_variable_values[3] = -2.641695639301903
    group_variable_values[4] = -0.21753881494171842
    group_variable_values[5] = 1.90281725288762
    group_variable_values[6] = 0.8802616816083866
    group.set_joint_value_target(group_variable_values)
    
    #homePt=geometry_msgs.msg.Pose()
    #homePt.position.x = 0.34942
    #homePt.position.y = 0.010376
    #homePt.position.z = 0.447501
    #homePt.orientation.x = 0.99924
    #homePt.orientation.y = 0.02179
    #homePt.orientation.z = 0.02585
    #homePt.orientation.w = 0.019214
    #group.set_pose_target(homePt)
    plan1 = group.plan()
    group.go(wait=True)
    group.clear_pose_targets()

# Transforms point from camera reference frame to world coordinates
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

    # Coordinated transformed from camera frame to world frame (panda_link0)
    p = listener.transformPoint("panda_link0",camframePt)   
    
    # Since the coordinates from the camera do not contain orientation information
    # either a custom RPY is provided or the orientation of the initial pose is used
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = 1
    pose_target.orientation.y = 0
    pose_target.orientation.z = 0
    pose_target.orientation.w = 0

    pose_target.position.x = p.point.x
    pose_target.position.y = p.point.y
    # 1 cm over target
    pose_target.position.z = p.point.z+0
    currentbox = [quat.x, quat.y, quat.z, quat.w]
    #addMarker(pose_targetFR,"panda_link0", False)
    return pose_target, quat, camframePt.point.x

# Takes in desire pose position and returns a pose message
def createpose(x,y,z,qx,qy,qz,qw):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = qx
    pose_target.orientation.y = qy
    pose_target.orientation.z = qz
    pose_target.orientation.w = qw
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    return pose_target

## Test functions ##

def performcalibration(calibration_pose):

    # Marker posted to view in Rviz
    addMarker(calibration_pose, "panda_link0")

    input("Review goal - Press to continue")

    # Move towards object/surface
    cartesian_traject(calibration_pose,0.05,0.5,True)

    input("Inspect relative orientation of end effector on bin plane")

    # Go up 5 cm before moving to next target
    calibration_pose.position.z += 0.05

    cartesian_traject(calibration_pose,0.05,0.5,True)

# A function to verify bin plane alignment with base frame
# Goes into every corner of the bin so the bin height can be adjusted.
# MIN/MAX points may vary
def binplanecalibration(height_at_bottom):

    # Mininum and maximum points of the box in world coordinates - Proceed with caution if box has been moved

    YMIN = -0.1139
    YMAX = 0.1838
    XMIN = 0.3341
    XMAX = 0.44374

    input("Review goal - Press to continue")

    # Far left
    group.clear_pose_targets()
    pose_targetFL = geometry_msgs.msg.Pose()
    pose_targetFL.orientation.x = 1
    pose_targetFL.orientation.y = 0
    pose_targetFL.orientation.z = 0
    pose_targetFL.orientation.w = 0
    pose_targetFL.position.x = XMAX
    pose_targetFL.position.y = YMAX
    pose_targetFL.position.z = height_at_bottom
    pose_goalFL=pose_targetFL

    #Far Right

    # Far right
    group.clear_pose_targets()
    pose_targetFR = geometry_msgs.msg.Pose()
    pose_targetFR.orientation.x = 1
    pose_targetFR.orientation.y = 0
    pose_targetFR.orientation.z = 0
    pose_targetFR.orientation.w = 0
    pose_targetFR.position.x = XMAX
    pose_targetFR.position.y = YMIN
    pose_targetFR.position.z = height_at_bottom
    pose_goalFR=pose_targetFR

    # Near right 

    group.clear_pose_targets()
    pose_targetNR = geometry_msgs.msg.Pose()
    pose_targetNR.orientation.x = 1
    pose_targetNR.orientation.y = 0
    pose_targetNR.orientation.z = 0
    pose_targetNR.orientation.w = 0
    pose_targetNR.position.x = XMIN
    pose_targetNR.position.y = YMIN
    pose_targetNR.position.z = height_at_bottom
    pose_goalNR=pose_targetNR

    # Near left 

    group.clear_pose_targets()
    pose_targetNL = geometry_msgs.msg.Pose()
    pose_targetNL.orientation.x = 1
    pose_targetNL.orientation.y = 0
    pose_targetNL.orientation.z = 0
    pose_targetNL.orientation.w = 0
    pose_targetNL.position.x = XMIN
    pose_targetNL.position.y = YMAX
    pose_targetNL.position.z = height_at_bottom
    pose_goalNL=pose_targetNL

    performcalibration(pose_goalFL)
    performcalibration(pose_goalFR)
    performcalibration(pose_goalNR)
    performcalibration(pose_goalNL)

    homePos()

    print("Finished calibration")

# Test funtion to see how much compression of the suction bell is needed to reach
# a target force. Suction cup is driven into a surface 1 mm at a time, first manually
# to find the point of contact and then automatically until maxforce is reached
def compressiontest(targetforce,maxforce,numiterations=10,bintest=False):

    # Mininum and maximum points of the box in world coordinates - Proceed with caution if box has been moved

    YMIN = -0.1139
    YMAX = 0.1838
    XMIN = 0.3341
    XMAX = 0.44374

    # Initial orientation used for the goal position
    (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time())

    # Orientation of end effector defined
    #q = quaternion_from_euler(math.pi, 0, 0)

    # A vector containing fistance from contact to targetforce
    compressiontotarget=[]
    approachrise=[]

    for i in range(numiterations):
        if(bintest==True):
            # A pose target created with random XY coordinates and constant z value  
            # of 0.03 (about 1 cm above bottom surface) within the bin boundaries
            group.clear_pose_targets()
            pose_targetFR = geometry_msgs.msg.Pose()
            pose_targetFR.orientation.x = 1
            pose_targetFR.orientation.y = 0
            pose_targetFR.orientation.z = 0
            pose_targetFR.orientation.w = 0
            pose_targetFR.position.x = np.random.uniform(XMIN, XMAX)
            pose_targetFR.position.y = np.random.uniform(YMIN, YMAX)
            pose_targetFR.position.z = 0.03
            pose_goal=pose_targetFR

        # Coordinates of an object recieved from find_pickpoint on Auður
        else:
            print("Waiting for pose goal")
            rospy.sleep(1)
            msg = rospy.wait_for_message("/pandaposition", PoseStamped)
            rospy.loginfo("Received at goal message!")
            addMarker(msg.pose, "camera_color_frame")
            if(msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0 ):
                while(msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0):
                    msg = rospy.wait_for_message("/pandaposition", PoseStamped)

            endPosition, _, _ = endPos(msg,rot) # transforming from camera frame to world frame

        pose_goal = endPosition
            
        # Marker posted to view in Rviz
        addMarker(pose_goal, "panda_link0")

        print(f'Pose goal #{i} of {numiterations} iterations')

        input("Review goal - Press to continue")

        # Move towards object/surface
        cartesian_traject(pose_goal,0.05,0.5,True)

        totdistance = 0
        # Vectors to collect force/distance da        forceval = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)

        # Vectors for needed data
        # Coordinate vectors to ensure no drift in xy position
        waypoints = []
        force = []
        distance = []

        #Setting initial values of force and distance vectors
        rospy.sleep(0.2)
        forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
        forceval=0
        initpos = pose_goal.position.z
        initforce = forcemsg.wrench.force.z
        increment = 0.001
        surfacereached=False
        targetforcereached=False
        surfacedist=0
        targetdistance=0

        # Check the pressure on the vaccuum
        #pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
        # Vacuum on
        #pubSuc.publish(Bool(True))

        while(totdistance < 0.05 and forceval < maxforce):
            pose_goal.position.z -= increment
            cartesian_traject(pose_goal,0.0005,0.3,True,1)
            
            #input("press to increment down")

            # Current Force value on the end effector acquired
            forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
            #pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
            forceval=forcemsg.wrench.force.z - initforce
            force.append(forceval)
            totdistance += increment 
            distance.append(totdistance)
            (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
            print(f'Force: {forceval} - Distance: {totdistance}')

            # Manual check if the surface has been reached
            if(surfacereached==False):
                surfacecheck=input("Enter to continue - press f if surface is reached")
                # If the surface is reached 
                if(surfacecheck=='f'):
                    surfacereached = True
                    #Approach distance
                    surfacedist=totdistance
                    # Total force rise in approach
                    approachforce=forceval-force[0]

            # Picking out at which position the target force is achieved
            if(targetforcereached==False and forceval >= targetforce):
                targetforcereached=True
                targetdistance=totdistance
                
        print(f'Finished #{i} iteration')

        # Splitting the vector by the following categories: Approach, compression, fully compressed.

        # Approach

        surfaceidx=distance.index(surfacedist)
        approachvec=np.asarray(distance[0:surfaceidx+1])
        approachF=np.asarray(force[0:surfaceidx+1])

        # Compression

        compresslength=14
        compressvec=np.asarray(distance[surfaceidx+1:surfaceidx+(compresslength+1)])
        compressF=np.asarray(force[surfaceidx+1:surfaceidx+(compresslength+1)])

        # Fixed vector to observe force from contact

        compressvecfix=np.asarray([i-surfacedist for i in compressvec])
        #compressvecfix=np.asarray(compressvecfix)

        # Fully compressed

        fcompressvec=np.asarray(distance[surfaceidx+(compresslength+1):])
        fcompressF=np.asarray(force[surfaceidx+(compresslength+1):])


        #pubSuc.publish(Bool(False))

        # distance = np.asarray(distance)
        # force = np.asarray(force)

        A = np.vstack([compressvecfix, np.ones(len(compressvecfix))]).T

        m, c = np.linalg.lstsq(A, compressF, rcond=None)[0]

        R = np.corrcoef(compressvec,compressF)

        #_ = plt.plot(distance, force, 'o', label='Force measurements', markersize=10)
        plt.figure()
        _ = plt.plot(approachvec, approachF, 'bo', label='Approach', markersize=10)
        _ = plt.plot(compressvec, compressF, 'yo', label='Suction compression', markersize=10)
        _ = plt.plot(fcompressvec, fcompressF, 'ro', label='Fully compressed', markersize=10)
        _ = plt.xlabel('Vertical distance (m)')
        _ = plt.ylabel('Measured force (N)')
        _ = plt.legend()
        plt.show()


        plt.figure()
        _ = plt.plot(compressvecfix, compressF, 'yo', label='Force/distance data', markersize=10)
        _ = plt.plot(compressvecfix, m*compressvecfix + c, 'r', label='Fitted line')
        _ = plt.xlabel('Compression distance (m)')
        _ = plt.ylabel('Measured force (N)')
        # _ = plt.text(0.5,0.8,f'{m}x+{c}', transform=ax.transAxes)
        _ = plt.legend()
        plt.show()

        # Needs plot of the linear part

        # pointval = m*0.008+c

        # xvec = np.asarray(xvec)
        # yvec = np.asarray(yvec)

        # print(xvec-xvec[0])
        # print(yvec-yvec[0])
        # print(realdist)
        # print(R)
        # print(pointval)
        # print(f'm: {m} - c: {c}')

        # Results appended to result vector
        compressdist=targetdistance-surfacedist
        compressiontotarget.append(compressdist)
        approachrise.append(approachforce)
        print(f'Compressed distance to target force: {compressdist}')
        input("Press to continue to next iteration")

        homePos()

    # Results from compression distance
    compresultarray=np.array(compressiontotarget)
    deviationdist=np.std(compresultarray)
    avgdist=np.mean(compresultarray)
    # Results from force rise in approach
    forceresultarray=np.array(approachrise)
    deviationforce=np.std(forceresultarray)
    avgforce=np.mean(forceresultarray)
    print(f'Average distance: {avgdist} - Standard deviation: {deviationdist}')
    print(f'Average approach force: {avgforce} - Standard deviation: {deviationforce}')

    plt.figure()
    _ = plt.plot(range(numiterations),compresultarray, 'o', label='', markersize=10)
    _ = plt.xlabel('Iteration')
    _ = plt.ylabel('Compression to target force (m)')
    _ = plt.legend()
    plt.show()

    plt.figure()
    _ = plt.plot(range(numiterations), forceresultarray, 'o', label='Fully compressed', markersize=10)
    _ = plt.xlabel('Iteration')
    _ = plt.ylabel('Force rise in approach')
    _ = plt.legend()
    plt.show()

# A test funcion to evaluate the performance of the measuring system
# The function takes in:
# - Initial height of the camera in world coordinates
# - Pose of the object of interest in world coordinates
# - Reference compression distance in meters of suction cup to travel to achieve 0.5 N force (0.004 recommended)
def measuredepth(camera_initial_height, object_pose, refdist):
    # The Position for measuring the depth at the bottom of the bin
    #endPosition, currbox, camdepth = endPos(msg,q)
    pose_targetFR = geometry_msgs.msg.Pose()
    pose_targetFR.orientation.x = object_pose.orientation.x
    pose_targetFR.orientation.y = object_pose.orientation.y
    pose_targetFR.orientation.z = object_pose.orientation.z
    pose_targetFR.orientation.w = object_pose.orientation.w
    pose_targetFR.position.x = object_pose.position.x
    pose_targetFR.position.y = object_pose.position.y
    pose_targetFR.position.z = object_pose.position.z+0.01
    pose_goal=pose_targetFR

    # Initialize position for while loop
    (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time())

    # Marker posted to view in Rviz
    addMarker(pose_goal, "panda_link0")

    input("Review goal - Press to continue")

    # Move towards object/surface
    cartesian_traject(pose_goal,0.05,0.5,True)

    input("Review position - Press to continue")

    totdistance = 0
    # Vectors to collect force/distance da        forceval = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)

    # Vectors for needed data
    waypoints = []
    realdist = []
    force = []
    distance = []

    #Setting initial values of force and distance vectors
    forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
    forceval=0
    initpos = pose_goal.position.z
    initforce = forcemsg.wrench.force.z
    increment = 0.001

    # Check the pressure on the vaccuum
    #pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
    # Vacuum on
    #pubSuc.publish(Bool(True))

    while(forceval < 0.5):
        pose_goal.position.z -= increment
        cartesian_traject(pose_goal,0.0005,0.5,True,0.8)
        
        #input("press to increment down")

        # Force value on the end effector acquired
        forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
        #pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
        forceval=forcemsg.wrench.force.z - initforce
        totdistance += increment 
        distance.append(totdistance)
        (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
        realdist.append(initpos-pose_goal.position.z)
        print(f'Force: {forceval}- Distance: {totdistance}')
        #print(f'{forceval.wrench.force.z}')

    print("Finished measuring")

    # Getting the position of the suction end in world coordinates
    sucendPt=PointStamped()
    sucendPt.header.frame_id = "panda_suction_end"
    grndtruthpos = listener.transformPoint("panda_link0",sucendPt)
    grndtruthheight=grndtruthpos.point.z

    # Experiments showed 0.5 N where achieved at 4 mm compression of suction cup
    grndtruthdepth=camera_initial_height-grndtruthheight+refdist

    homePos()

    return grndtruthdepth

def testmeasuredepth():

    input("Remove any objects in the bin - Press enter to continue")

    # Use initial orientation for the pose goals
    (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time())

    # Getting the position of the camera in world coordinates
    camPt=PointStamped()
    camPt.header.frame_id = "camera_color_frame"
    camInitial = listener.transformPoint("panda_link0",camPt)
    initialheight=camInitial.point.z

    print(f'Initial height of camera: {initialheight}')

    # The Position for measuring the depth at the bottom of the bin
    #endPosition, currbox, camdepth = endPos(msg,q)
    pose_targetFR = geometry_msgs.msg.Pose()
    pose_targetFR.orientation.x = 1
    pose_targetFR.orientation.y = 0
    pose_targetFR.orientation.z = 0
    pose_targetFR.orientation.w = 0
    pose_targetFR.position.x = 0.38946
    pose_targetFR.position.y = 0.0297
    pose_targetFR.position.z = 0.02539

    # Getting depth of the bin
    bindepth=measuredepth(initialheight,pose_targetFR,0.004)

    print(f'Depth of bin: {bindepth}')

    # Getting current pose and moving up 10 cm
    # currpose=group.get_current_pose()

    # pose_targetFR=pose_unstamped(currpose)

    # pose_targetFR.position.z=pose_targetFR.position.z+0.15
    # pose_targetFR.position.y=pose_targetFR.position.y

    # Marker posted to view in Rviz
    # addMarker(pose_targetFR, "panda_link0")

    # input("Review goal - Press to continue")

    # For testing purposes
    #cartesian_traject(pose_targetFR,0.05,0.5,True, 0.1)

    # Inspect plane bottom
    #input("Inspect bin plane")

    homePos()

    # Fixing home position, unrelated to the function
    #pose_target=createpose(0.318,-0.0364,0.25,1,0,0,0)

    #addMarker(pose_target,"panda_link0")

    #input("Review pose")

    #cartesian_traject(pose_target,0.05,0.5,True, 0.1)

    input("Place object in the center of the bin - Press enter to continue")

    # # The Position for measuring the depth at the bottom of the bin
    # #endPosition, currbox, camdepth = endPos(msg,q)


    # Getting position from camera
    msg = rospy.wait_for_message("/pandaposition", PoseStamped)

    i=0

    while(msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0 ):
        msg = rospy.wait_for_message("/pandaposition", PoseStamped)
        i+=1
        if(i==1000):
            print("Waiting for object.....")
            i=0

    pose_targetFR,_,camdepth=endPos(msg,rot)

    print(f'Object depth estimated by camera: {camdepth}')

    # Calculated object width from camera using measured bin depth
    camwidth=bindepth-camdepth

    objectdepth=measuredepth(initialheight,pose_targetFR,0.004)

    print(f'Depth of object: {objectdepth}')

    objectthickness= bindepth-objectdepth

    print(f'Measured thickness of object: {objectthickness}')
    print(f'Estimated from camera: {camwidth}')


# A function to test the performance of measuring using Franka Panda
# Original function that most of the others are built on, kept if 
# something gets lost
def measuretest():

    # Getting position of objects
    #msg = rospy.wait_for_message("/pandaposition", PoseStamped)
    #addMarker(msg.pose, "camera_color_frame")

    # Use initial orientation for the pose goals
    (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time())

    # Getting the position of the camera in world coordinates
    camPt=PointStamped()
    camPt.header.frame_id = "camera_color_frame"
    camInitial = listener.transformPoint("panda_link0",camPt)
    initialheight=camInitial.point.z

    # Saving estimated depth from L515 camera
    # lidardepth=msg.pose.position.x

    print(f'Initial height of camera: {initialheight}')

    # Orientation of end effector defined
    #q = quaternion_from_euler(math.pi, 0, 0)

    # The Position for measuring the depth at the bottom of the bin
    #endPosition, currbox, camdepth = endPos(msg,q)
    pose_targetFR = geometry_msgs.msg.Pose()
    pose_targetFR.orientation.x = 1
    pose_targetFR.orientation.y = 0
    pose_targetFR.orientation.z = 0
    pose_targetFR.orientation.w = 0
    pose_targetFR.position.x = 0.4495
    pose_targetFR.position.y = -0.04686
    pose_targetFR.position.z = 0.0276
    pose_goal=pose_targetFR

    # Point transformed to world frame - Near left
    #endPosition, currbox, camdepth = endPos(msg,q)
    # pose_targetFR = geometry_msgs.msg.Pose()
    # pose_targetFR.orientation.x = 1
    # pose_targetFR.orientation.y = 0
    # pose_targetFR.orientation.z = 0
    # pose_targetFR.orientation.w = 0
    # pose_targetFR.position.x = 0.38055
    # pose_targetFR.position.y = 0.16
    # pose_targetFR.position.z = 0.0222
    # pose_goal=pose_targetFR

    # Marker posted to view in Rviz
    addMarker(pose_goal, "panda_link0")

    input("Review goal - Press to continue")

    # Move towards object/surface
    cartesian_traject(pose_goal,0.05,0.5,True)

    totdistance = 0
    # Vectors to collect force/distance da        forceval = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)

    # Vectors for needed data
    xvec = []
    yvec = []
    waypoints = []
    realdist = []
    force = []
    distance = []

    #Setting initial values of force and distance vectors
    forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
    forceval=0
    initpos = pose_goal.position.z
    initforce = forcemsg.wrench.force.z
    increment = 0.001

    # Check the pressure on the vaccuum
    #pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
    # Vacuum on
    #pubSuc.publish(Bool(True))

    while(trans.position.z < 0.005 and forceval < 0.5):
        pose_goal.position.z -= increment
        cartesian_traject(pose_goal,0.0005,0.5,True)
        
        #input("press to increment down")

        # Force value on the end effector acquired
        forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
        #pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
        forceval=forcemsg.wrench.force.z - initforce
        force.append(forceval)
        totdistance += increment 
        distance.append(totdistance)
        (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
        xvec.append(trans[0])
        yvec.append(trans[1])
        realdist.append(initpos-pose_goal.position.z)
        print(f'Force: {forceval}- pressure: {pressure} - Distance: {totdistance}')
        #print(f'{forceval.wrench.force.z}')

    print("Finished measuring")

    # Getting the position of the suction end in world coordinates
    sucendPt=PointStamped()
    sucendPt.header.frame_id = "panda_suction_end"
    grndtruthpos = listener.transformPoint("panda_link0",sucendPt)
    grndtruthheight=grndtruthpos.point.z

    # Experiments showed 0.5 N where achieved at 3 mm compression of suction cup
    grndtruthdepth=initialheight-grndtruthheight+0.003

    #print(f'Lidar depth: {lidardepth} - Measured depth: {grndtruthdepth}')
    print(f'Measured depth: {grndtruthdepth}')


    #pubSuc.publish(Bool(False))

    # distance = np.asarray(distance)
    # force = np.asarray(force)

    # A = np.vstack([distance, np.ones(len(distance))]).T

    # m, c = np.linalg.lstsq(A, force, rcond=None)[0]

    # R = np.corrcoef(distance,force)

    # _ = plt.plot(distance, force, 'o', label='Force measurements', markersize=10)
    # #_ = plt.plot(distance, m*distance + c, 'r', label='Fitted line')
    # _ = plt.xlabel('Suction cup compression (m)')
    # _ = plt.ylabel('Contact force (N)')
    # _ = plt.legend()
    # plt.show()

    # pointval = m*0.008+c

    # xvec = np.asarray(xvec)
    # yvec = np.asarray(yvec)

    # print(xvec-xvec[0])
    # print(yvec-yvec[0])
    # print(realdist)
    # print(R)
    # print(pointval)
    # print(f'm: {m} - c: {c}')


def runstuff():
    # Target force - maximum force - iterations
    #compressiontest(0.5,3,10)
    testmeasuredepth()
    #binplanecalibration(0.03)


def main():
    try:
        runstuff()
    except KeyboardInterrupt:
        return


if __name__ == '__main__':

    #rostopic pub  std_msgs/Bool true --once
    #pubSuc = rospy.Publisher('/vacuum/set_suction_on', Bool, queue_size=1) # Talk to the festo ovem suction cup
    rospy.init_node('pick_and_place')
    listener = tf.TransformListener()
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("panda_arm")
    group.allow_replanning(True)
    group.set_planning_time(1)
    group.set_goal_position_tolerance(0.0001)
    group.set_goal_orientation_tolerance(0.001)
    group.set_goal_joint_tolerance(0.0001)
    group.set_num_planning_attempts(3)
    ref=group.get_pose_reference_frame()
    eef_link=group.get_end_effector_link()
    goaltol = group.get_goal_position_tolerance()
    orientol = group.get_goal_orientation_tolerance()
    jointtol= group.get_goal_joint_tolerance()
    print(f'Reference frame: {ref}')
    print(f'End effector link: {eef_link}')
    print(f'Goal tolerance: {goaltol}')
    print(f'Orientation tolerance: {orientol}')
    print(f'joint tolerance: {jointtol}')
    # A publisher to publish pose goal markers which can be viewed in Rviz
    markpub = rospy.Publisher("posmarker", Marker, queue_size=1)

    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10) # Talks to the moveit motion planner
    homePos()

    main()

    #while not rospy.is_shutdown():
    #    currstate = stateMachine(currstate)