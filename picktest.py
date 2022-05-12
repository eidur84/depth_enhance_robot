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
#folder_path = "/mnt/bigdata/eidur14/data"
folder_path = "/home/lab/Documents/Eidur/Myndir/"
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
def cartesian_traject(waypoint,step,vel_scale=0.5,pause=False,sleeptime=0.6):
    
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

def plot_bar(data,title):
    plt.bar(*np.unique(data, return_counts=True),edgecolor='black',color='blue',linewidth=1,zorder=3)
    plt.xlabel('Compression to target pressure (mm)')
    plt.ylabel('Number of occurences')
    plt.grid(axis='y',color='black',linestyle='--',linewidth=0.2, zorder=0)
    plt.xticks(np.arange(-4, 12, step=1))  # Set label locations.
    plt.yticks(np.arange(0, 5, step=1))  # Set label locations.
    plt.title(title)
    plt.show()

def shuffleobjects(safemode=True): #move the item that is picked to a "locked" location...

    # Getting current pose and transforming it to non-stamped pose
    currentp=group.get_current_pose()
    currentp=pose_unstamped(currentp)

    # Go up 10 cm
    currentp.position.z += 0.05

    if(safemode):
        addMarker(currentp, "panda_link0")

        input("Review marker - Press to continue")

    cartesian_traject(currentp, 0.05,0.5,pause=True)

    # Add a random orientation
    q = quaternion_from_euler(math.pi, 0, np.random.uniform(-math.pi/8, math.pi/8))

    #x = currentp.position.x
    # Try to keep object around middle
    #x = np.random.uniform(XMIN+0.03, XMAX-0.04)
    x = 0.39
    y = np.random.uniform(YMIN+0.04, YMAX-0.04)
    z = currentp.position.z

    pose_target=createpose(x,y,z,q[0],q[1],q[2],q[3])

    if(safemode):
        addMarker(pose_target, "panda_link0")
        input("Review marker - Press to continue")
    
    cartesian_traject(pose_target, 0.05,0.5,pause=True)
    # Shut of vacuum
    pubSuc.publish(Bool(False))

# Moves down vertically towards a pick point, takes in a force goal as a stopping condition indicating the pick point has been reached
def pick(pressure_goal,debug=True):
    
    # Getting current pose and transforming it to non-stamped pose
    currentp=group.get_current_pose()
    currentp=pose_unstamped(currentp)
    
    # The total vertical movement and pick boolean initialized
    totdistance = 0
    pick=False

    # Counting number of increments to suction seal
    count=0

    # Vacuum started
    pubSuc.publish(Bool(True))

    rospy.sleep(0.1)

    # Vaccum pressure checked
    pressure = rospy.wait_for_message("/vacuum/pressure", Float32)

    #Setting initial values of force and distance vectors
    initpos = currentp.position.z
    increment = 0.001

    # Moves down 1 cm to try to get a pick. Pressure trigger is at -0.1 bar gauge pressure
    while(totdistance < 0.01 and pressure.data > pressure_goal):
        currentp.position.z -= increment
        count += 1
        cartesian_traject(currentp,0.0005,0.5,True)
        
        #input("press to increment down")
        pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
        totdistance += increment 
        if(debug):
            print(f'Pressure: {pressure.data} - Distance: {totdistance}')
            #input("Press to continue")
        # If suction is achieved set pick to true

    if pressure.data <= -0.1:
        pick=True
    else:
        # If vacuum seal not achieved turn of vacuum
        pubSuc.publish(Bool(False))

    
    return pick, count

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
def endPos(msg): 
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
    pose_target.position.z = p.point.z+0.01
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

    YMIN = -0.12404
    YMAX = 0.1694
    XMIN = 0.3154
    XMAX = 0.4883

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

    # Middle 

    group.clear_pose_targets()
    pose_targetM = geometry_msgs.msg.Pose()
    pose_targetM.orientation.x = 1
    pose_targetM.orientation.y = 0
    pose_targetM.orientation.z = 0
    pose_targetM.orientation.w = 0
    pose_targetM.position.x = 0.39488
    pose_targetM.position.y = 0.02177
    pose_targetM.position.z = height_at_bottom
    pose_goalM=pose_targetM

    performcalibration(pose_goalM)
    performcalibration(pose_goalFL)
    performcalibration(pose_goalFR)
    performcalibration(pose_goalNR)
    performcalibration(pose_goalNL)

    homePos()

    print("Finished calibration")

# Test funtion to see how much compression of the suction bell is needed to reach
# a target force. Suction cup is driven into a surface 1 mm at a time, first manually
# to find the point of contact and then automatically until maxforce is reached
def pressuretest(targetpressure,numiterations=10):


    # A vector containing fistance from contact to targetforce
    compressiontotarget=[]


    for i in np.arange(numiterations):




        # Coordinates of an object recieved from find_pickpoint on Auður
        
        print("Waiting for pose goal")
        rospy.sleep(1)
        msg = rospy.wait_for_message("/pandaposition", PoseStamped)
        rospy.loginfo("Received at goal message!")
        addMarker(msg.pose, "camera_color_frame")
        
        while(msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0):
            msg = rospy.wait_for_message("/pandaposition", PoseStamped)

        endPosition, _, _ = endPos(msg) # transforming from camera frame to world frame

        # Filtering out bad pick points
        while(endPosition.position.z > 0.15):
            msg = rospy.wait_for_message("/pandaposition", PoseStamped)
            endPosition, _, _ = endPos(msg) # transforming from camera frame to world frame


        pose_goal = endPosition
            
        # Marker posted to view in Rviz
        addMarker(pose_goal, "panda_link0")

        print(f'Pose goal #{i+1} of {numiterations} iterations')

        input("Review goal - Press to continue")

        # Move towards object/surface
        cartesian_traject(pose_goal,0.05,0.5,True)

        totdistance = 0
        distance=[]
        force=[]
        # Vectors to collect force/distance da        forceval = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)

        #Setting initial values of force and distance vectors
        rospy.sleep(2)
        forceval=0
        initpos = pose_goal.position.z
        increment = 0.001
        surfacereached=False
        targetpressurereached=False
        surfacedist=0
        targetdistance=0

        forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
        forceval=0
        initpos = pose_goal.position.z
        initforce = forcemsg.wrench.force.z

        # Check the pressure on the vaccuum
        pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
        pressureval=pressure.data
        # Vacuum on
        #pubSuc.publish(Bool(True))

        while(totdistance < 0.04 and pressureval >= targetpressure):
            pose_goal.position.z -= increment
            cartesian_traject(pose_goal,0.0005,0.1,True,1)
            
            #input("press to increment down")

            forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
            forceval=forcemsg.wrench.force.z - initforce

            # Current Force value on the end effector acquired
            pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
            pressureval=pressure.data
            totdistance += increment
            distance.append(totdistance)
            force.append(forceval)
            print(f'Pressure: {pressure} - Force: {forceval} Distance: {totdistance}')

            # Manual check if the surface has been reached
            if(surfacereached==False):
                surfacecheck=input("Press f if surface is reached\n")
                # If the surface is reached 
                if(surfacecheck=='f'):
                    surfacereached = True
                    #Approach distancem, converted to mm
                    surfacedist=totdistance
                    # Turn on vacuum generator at surface
                    pubSuc.publish(Bool(True))
          
        print(f'Finished #{i+1} iteration')

        # Total distance to target pressure
        targetdistance=totdistance
        surfacedist=surfacedist

        surfaceidx=distance.index(surfacedist)
        targetdistidx=distance.index(targetdistance)
        #targetdistance=targetdistance*1000

        approachvec=np.asarray(distance[0:surfaceidx])
        approachvec=approachvec*1000
        approachF=np.asarray(force[0:surfaceidx])

        # Compression

        compressvec=np.asarray(distance[surfaceidx:-1])
        compressvec=compressvec*1000
        compressF=np.asarray(force[surfaceidx:-1])

        pubSuc.publish(Bool(False))

        # Results appended to result vector and convert to mm
        compressdist=(targetdistance-surfacedist)*1000
        compressiontotarget.append(compressdist)
        print(f'Compressed distance to target pressure: {compressdist}')

        _ = plt.plot(approachvec, approachF, 'bo', label='Approach', markersize=10)
        _ = plt.plot(compressvec, compressF, 'yo', label='Suction compression', markersize=10)
        _ = plt.xlabel('Distance (mm)')
        _ = plt.ylabel('Force (N)')
        _ = plt.legend()
        plt.show()

        input("Press to continue to next iteration")

        homePos()

    # Results from compression distance
    compresultarray=np.array(compressiontotarget)
    deviationdist=np.std(compresultarray)
    avgdist=np.mean(compresultarray)
    
    print(f'Average distance: {avgdist} - Standard deviation: {deviationdist}')

    plt.figure()
    plt.bar(*np.unique(compresultarray, return_counts=True))
    _ = plt.xlabel('Compression to target pressure (mm)')
    _ = plt.ylabel('Number of occurences')
    plt.show()

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

    for i in np.arange(numiterations):
        if(bintest==True):
            # A pose target created with random XY coordinates and constant z value  
            # of 0.03 (about 1 cm above bottom surface) within the bin boundaries
            # group.clear_pose_targets()
            # pose_targetFR = geometry_msgs.msg.Pose()
            # pose_targetFR.orientation.x = 1
            # pose_targetFR.orientation.y = 0
            # pose_targetFR.orientation.z = 0
            # pose_targetFR.orientation.w = 0
            # pose_targetFR.position.x = np.random.uniform(XMIN, XMAX)
            # pose_targetFR.position.y = np.random.uniform(YMIN, YMAX)
            # pose_targetFR.position.z = 0.03
            # pose_goal=pose_targetFR

            pose_goal=createpose(np.random.uniform(XMIN, XMAX),np.random.uniform(YMIN, YMAX),0.03,1,0,0,0)



        # Coordinates of an object recieved from find_pickpoint on Auður
        else:
            print("Waiting for pose goal")
            rospy.sleep(1)
            msg = rospy.wait_for_message("/pandaposition", PoseStamped)
            rospy.loginfo("Received at goal message!")
            addMarker(msg.pose, "camera_color_frame")
           
            while(msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0):
                msg = rospy.wait_for_message("/pandaposition", PoseStamped)

            endPosition, _, _ = endPos(msg) # transforming from camera frame to world frame

        if(bintest==False):
            pose_goal = endPosition
            
        # Marker posted to view in Rviz
        addMarker(pose_goal, "panda_link0")

        print(f'Pose goal #{i+1} of {numiterations} iterations')

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
        coef=[]
        const=[]

        #Setting initial values of force and distance vectors
        rospy.sleep(2)
        forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
        forceval=0
        initpos = pose_goal.position.z
        initforce = forcemsg.wrench.force.z
        increment = 0.001
        surfacereached=False
        #targetforcereached=False
        surfacedist=0
        targetdistance=0

        # Check the pressure on the vaccuum
        pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
        # Vacuum on
        #pubSuc.publish(Bool(True))

        while(totdistance < 0.06 and forceval < maxforce):
        #while(totdistance < 0.06 and forceval < maxforce):
            pose_goal.position.z -= increment
            cartesian_traject(pose_goal,0.0005,0.1,True,1)
            
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
                surfacecheck=input("Press f if surface is reached\n")
                # If the surface is reached 
                if(surfacecheck=='f'):
                    surfacereached = True
                    #Approach distancem, converted to mm
                    surfacedist=totdistance
                    # Total force rise in approach
                    approachforce=forceval

            # Picking out at which position the target force is achieved
            if(targetforcereached==False and forceval >= targetforce):
                targetforcereached=True
                targetdistance=totdistance
                
        print(f'Finished #{i} iteration')

        # Splitting the vector by the following categories: Approach, compression, fully compressed.

        # Approach

        surfaceidx=distance.index(surfacedist)
        targetdistidx=distance.index(targetdistance)
        targetdistance=targetdistance*1000
        surfacedist=surfacedist*1000
        approachvec=np.asarray(distance[0:surfaceidx])
        approachvec=approachvec*1000
        approachF=np.asarray(force[0:surfaceidx])

        # Compression

        compresslength=15
        compressvec=np.asarray(distance[surfaceidx:surfaceidx+(compresslength+1)])
        compressvec=compressvec*1000
        compressF=np.asarray(force[surfaceidx:surfaceidx+(compresslength+1)])

        # Fixed vector to observe force from contact

        compressvecfix=np.asarray(compressvec)
        compressvecfix=compressvecfix-surfacedist
        #compressvecfix=np.asarray(compressvecfix)

        # Fully compressed

        fcompressvec=np.asarray(distance[surfaceidx+(compresslength+1):])
        fcompressvec=fcompressvec*1000
        fcompressF=np.asarray(force[surfaceidx+(compresslength+1):])

        # Target force compresion



        #pubSuc.publish(Bool(False))

        # distance = np.asarray(distance)
        # force = np.asarray(force)

        A = np.vstack([compressvecfix, np.ones(len(compressvecfix))]).T

        m, c = np.linalg.lstsq(A, compressF, rcond=None)[0]

        R = np.corrcoef(compressvec,compressF)

        # _ = plt.plot(distance, force, 'o', label='Force measurements', markersize=10)
        plt.figure()
        _ = plt.plot(approachvec, approachF, 'bo', label='Approach', markersize=10)
        _ = plt.plot(compressvec, compressF, 'yo', label='Suction compression', markersize=10)
        _ = plt.plot(fcompressvec, fcompressF, 'ro', label='Fully compressed', markersize=10)
        _ = plt.plot(targetdistance, force[targetdistidx], 'kx', label='Target force', markersize=10)
        _ = plt.xlabel('Vertical distance (mm)')
        _ = plt.ylabel('Measured force (N)')
        _ = plt.legend()
        plt.show()


        plt.figure()
        _ = plt.plot(compressvecfix, compressF, 'yo', label='Force/distance data', markersize=10)
        _ = plt.plot(compressvecfix, m*compressvecfix + c, 'r', label='Fitted line')
        _ = plt.xlabel('Compression distance (mm)')
        _ = plt.ylabel('Measured force (N)')
        _ = plt.text(0.1,1.4,f'{m:.3f}x+({c:.3f})',fontsize=12)
        _ = plt.text(0.1,1.2,f'R={R[0,1]:.3f}',fontsize=12)
        _ = plt.legend()
        plt.show()

        print(R)

        print(f'm: {m}x + {c}')

        # Results appended to result vector
        compressdist=(targetdistance-surfacedist)
        compressiontotarget.append(compressdist)
        approachrise.append(approachforce)
        coef.append(m)
        const.append(c)
        print(coef,const)
        print(f'Compressed distance to target force: {compressdist}')
        input("Press to continue to next iteration")

        homePos()

    # Results from compression distance
    compresultarray=np.array(compressiontotarget)
    deviationdist=np.std(compresultarray)
    avgdist=np.mean(compresultarray)
    # Compression distance with outliers removed (Filter out all values 0 or below)
    filteridx=np.where(compresultarray > 0)
    filteredcompresaultarray=compresultarray[filteridx]
    avgdistfiltered=np.mean(filteredcompresaultarray)
    deviationfiltered=np.std(filteredcompresaultarray)

    # Results from force rise in approach
    forceresultarray=np.array(approachrise)
    deviationforce=np.std(forceresultarray)
    avgforce=np.mean(forceresultarray)
    # Results from linear equation on compression part
    coef=np.asarray(coef)
    const=np.asarray(const)
    avgcoef=np.mean(coef)
    stdcoef=np.std(coef)
    avgconst=np.mean(const)
    stdconst=np.std(const)
    print(f'Average distance: {avgdist} - Standard deviation: {deviationdist}')
    print(f'Average distance filtered: {avgdist} - Standard deviation filtered: {deviationdist}')
    print(f'Average approach force: {avgforce} - Standard deviation: {deviationforce}')
    print(f'Averaged linear equation for compression: {avgcoef}x+{avgconst}')
    print()

    plt.figure()
    plt.bar(*np.unique(compresultarray, return_counts=True))
    _ = plt.xlabel('Compression to target force (mm)')
    _ = plt.ylabel('Number of occurences')
    plt.show()

    # plt.figure()
    # plt.bar(*np.unique(forceresultarray, return_counts=True))
    # _ = plt.xlabel('Number of instances')
    # _ = plt.ylabel('Force in approach')
    # plt.show()

# A test funcion to evaluate the performance of the measuring system
# The function takes in:
# - Initial height of the camera in world coordinates
# - Pose of the object of interest in world coordinates
# - Reference compression distance in meters of suction cup to travel to achieve 0.5 N force (0.004 recommended)
def measuredepthF(camera_initial_height, object_pose, refdist):

    # Pose goal at the object
    pose_goal=object_pose

    # Initialize position of panda suction end for the while loop
    (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time())

    # Marker posted to view in Rviz
    #addMarker(pose_goal, "panda_link0")

    #input("Review goal - Press to continue")

    # Move towards object/surface
    cartesian_traject(pose_goal,0.05,0.5,True)

    #input("Review position - Press to continue")

    totdistance = 0
    # Vectors to collect force/distance da        forceval = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)

    # Vectors for needed data
    waypoints = []
    realdist = []
    force = []
    distance = []

    #Setting initial values of force and distance vectors
    rospy.sleep(1)
    forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
    forceval=0
    initpos = pose_goal.position.z
    initforce = forcemsg.wrench.force.z
    increment = 0.001

    # Check the pressure on the vaccuum
    #pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
    # Vacuum on
    #pubSuc.publish(Bool(True))

    # Drives down while force trigger has not been reached
    while(trans[2] > 0.02 and forceval < 0.5):
        pose_goal.position.z -= increment
        # waypoint,step,vel_scale=0.5,pause=False,sleeptime=0.6
        cartesian_traject(pose_goal,0.0005,0.1,True,0.8)
        
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

    print(f'Height of object: {grndtruthheight}')

    # Experiments showed 0.5 N where achieved at 4 mm compression of suction cup
    grndtruthdepth=camera_initial_height-(grndtruthheight+refdist)

    return grndtruthdepth

# A test funcion to evaluate the performance of the measuring system
# The function takes in:
# - Initial height of the camera in world coordinates
# - Pose of the object of interest in world coordinates
# - Reference compression distance in meters of suction cup to travel to achieve 0.5 N force (0.004 recommended)
def measuredepthP(camera_initial_height, object_pose, refdist):

    # Pose goal at the object
    pose_goal=object_pose

    # Initialize position of panda suction end for the while loop
    (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time())

    # Marker posted to view in Rviz
    #addMarker(pose_goal, "panda_link0")

    #input("Review goal - Press to continue")

    # Move towards object/surface
    cartesian_traject(pose_goal,0.05,0.5,True)

    #input("Review position - Press to continue")

    totdistance = 0
    # Vectors to collect force/distance da        forceval = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)

    # Vectors for needed data
    waypoints = []
    realdist = []
    force = []
    distance = []

    #Setting initial values of force and distance vectors
    rospy.sleep(1)
    forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
    forceval=0
    initpos = pose_goal.position.z
    initforce = forcemsg.wrench.force.z
    increment = 0.001

    # Boolean for pick success
    pick=False

    # Vacuum on and execution halted for 300 ms
    pubSuc.publish(Bool(True))
    rospy.sleep(0.1)

    # Check the pressure on the vaccuum
    pressure = rospy.wait_for_message("/vacuum/pressure", Float32)

    # Drives down while force pressure has not been reached. Maximum approach 2,5 cm
    while(totdistance < 0.025 and pressure.data > -0.1):
        pose_goal.position.z -= increment
        # waypoint,step,vel_scale=0.5,pause=False,sleeptime=0.6
        cartesian_traject(pose_goal,0.0005,0.1,True,0.8)
        #input("press to increment down")
        rospy.sleep(0.05)
        # Force value on the end effector acquired
        pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
        totdistance += increment 
        #(trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
        print(f'Pressure: {pressure.data}- Distance: {totdistance}')
        

    print("Finished measuring")

    if pressure.data < -0.1:
        pick=True
    else:
        pubSuc.publish(Bool(False))
        
    # Getting the position of the suction end in world coordinates
    sucendPt=PointStamped()
    sucendPt.header.frame_id = "panda_suction_end"
    grndtruthpos = listener.transformPoint("panda_link0",sucendPt)
    grndtruthheight=grndtruthpos.point.z

    print(f'Height of object: {grndtruthheight}')

    # Experiments showed 0.5 N where achieved at 4 mm compression of suction cup
    grndtruthdepth=camera_initial_height-(grndtruthheight+refdist)

    return grndtruthdepth, pick

# Measures depth of an object using known values for object and height of bin in base frame (panda_link0)
# The real depth is thus considered known.
def testmeasuredepth(iter,ground_truth_width,groundtruth_binheight, product,forcemeasure=True):

    if forcemeasure:
        forceorpressure="force"
    else:
        forceorpressure="pressure"

    # Filename related
    suffix = datetime.now().strftime("%y%m%d_%H%M%S")
    path=folder_path+"/Depthmeasuretest"
    imgpath = os.path.join(path,"Depthmeasure"+forceorpressure+product+suffix+".png")
    csvpath= os.path.join(path,"Depthmeasure"+forceorpressure+".csv")
    # Getting the position of the camera in world coordinates
    camPt=PointStamped()
    camPt.header.frame_id = "camera_color_frame"
    camInitial = listener.transformPoint("panda_link0",camPt)
    initialheight=camInitial.point.z

    # Ground truth depth, converted to mm

    grndtrthdepth=(initialheight-ground_truth_width-groundtruth_binheight)

    # Camera depth and measured depth collected for each iteration
    camdeptharr=[]
    measureddeptharr=[]
    picklengtharr=[]
    pickval=False

    for i in np.arange(iter):

        # Getting the position of the camera in world coordinates
        camPt=PointStamped()
        camPt.header.frame_id = "camera_color_frame"
        camInitial = listener.transformPoint("panda_link0",camPt)
        initialheight=camInitial.point.z

        print(f'Initial height of camera: {initialheight}')

        #input("Place object in the center of the bin - Press enter to continue")

        rospy.sleep(2)

        # Getting position from camera
        msg = rospy.wait_for_message("/pandaposition", PoseStamped)

        i=0

        while(msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0 ):
            msg = rospy.wait_for_message("/pandaposition", PoseStamped)
            i+=1
            if(i==1000):
                print("Waiting for object.....")
                i=0

        pose_target,_,camdepth=endPos(msg)

        print(f'Object depth estimated by camera: {camdepth}')

        # Calculated object width from camera using measured bin depth
        #camwidth=bindepth-camdepth

        if forcemeasure:
            objectdepth=measuredepthF(initialheight,pose_target,0.003)
        else:
            objectdepth,pickval=measuredepthP(initialheight,pose_target,0.002)

        print(f'Depth of object: {objectdepth}')
        print(f'Measurement error: {objectdepth-grndtrthdepth}')

        #objectwidth= bindepth-objectdepth

        print(f'Estimated from camera: {camdepth}')
        print(f'Camera error: {camdepth-grndtrthdepth}')

        camdeptharr.append(camdepth)
        measureddeptharr.append(objectdepth)

        if forcemeasure:
            # Trying to pick up the object with a pressure trigger at -0.1 bar
            pickval,pick_increments=pick(-0.1)
            picklengtharr.append(pick_increments)

        if pickval:
            shuffleobjects(safemode=False)

        homePos()

    # Converting ground truth to mm
    grndtrthdepth=(initialheight-ground_truth_width-groundtruth_binheight)*1000

    # Converting to np arrays
    camdeptharr=np.asarray(camdeptharr)*1000
    measureddeptharr=np.asarray(measureddeptharr)*1000

    # Calculating all errors
    errordeptharr=measureddeptharr-grndtrthdepth
    avgmeasurederror_depth=np.mean(errordeptharr)
    stdmeasurederror_depth=np.std(errordeptharr)

    # Filtering out error values above 4 (Since they can be filtered out using suction)
    # errordeptharr_fixed=errordeptharr[np.where(errordeptharr >= -4)]
    # avgmeasurederror_fixed=np.mean(errordeptharr_fixed)
    # stdmeasurederror_fixed=np.std(errordeptharr_fixed)
    # avgaccuracy_fixed = (1-abs(avgmeasurederror_fixed/grndtrthdepth))*100

    print(errordeptharr)
    print(errordeptharr_fixed)

    print(picklengtharr)

    avgmeasured_depth=np.mean(measureddeptharr)
    stdmeasured_depth=np.std(measureddeptharr)
    avgmeasured_cam=np.mean(camdeptharr)
    stdmeasured_cam=np.std(camdeptharr)

    avgerror_robot=avgmeasured_depth-grndtrthdepth
    avgerror_cam=avgmeasured_cam-grndtrthdepth

    avgaccuracy_robot = (1-abs(avgerror_robot/grndtrthdepth))*100
    avgaccuracy_cam = (1-abs(avgerror_cam/grndtrthdepth))*100

    plotdata=np.unique(errordeptharr, return_counts=True)

    plt.figure()
    plt.hist(errordeptharr,edgecolor='black',linewidth=1,zorder=3)
    plt.title(f'Depth measure {forceorpressure} test: {product}')
    plt.grid(axis='y',color='black',linestyle='--',linewidth=0.2, zorder=0)
    _ = plt.xlabel('Measurement error (mm)')
    _ = plt.ylabel('Number of occurences')
    plt.savefig(imgpath)
    plt.show()

    # Data for csv prepared
    list_row_append = [(f'{product}',avgmeasurederror_depth,avgaccuracy_robot,stdmeasured_depth,avgaccuracy_cam,stdmeasured_cam)]
 
    dtype = [('Name', (np.str)), ('Average error', np.float), ('Average accuracy', np.float),('Repeatability', np.float),('Camera accuracy', np.float),('Camera repeatability', np.float)]
  
    data = np.array(list_row_append, dtype=dtype)

    #data=np.asarray(["D:Fi",avgmeasurederror_depth,avgaccuracy_robot,stdmeasured_depth,avgaccuracy_cam,stdmeasured_cam])

    # Write the results to a csv
    with open(csvpath,'a+') as csvfile:
        np.savetxt(csvfile,data,delimiter=',',fmt=['%s' , '%.2f', '%.2f','%.2f','%.2f','%.2f'], comments='')

    print(f'Avg error robot: {avgmeasurederror_depth} - std error: {stdmeasurederror_depth} mm')
    #print(f'Avg error robot fixed: {avgmeasurederror_fixed} - std error fixed: {stdmeasurederror_fixed} mm')
    print(f'Avg percent accuracy robot fixed: {avgaccuracy_fixed} % - repeatability: {stdmeasurederror_fixed} mm')
    print(f'Avg percent accuracy robot: {avgaccuracy_robot} % - repeatability: {stdmeasured_depth} mm')
    print(f'Avg percent accuracy camera: {avgaccuracy_cam} % - repeatability: {stdmeasured_cam} mm')

def runstuff():
    # Target force - maximum force - iterations
    # target force - Max force - Iterations
    #compressiontest(0.5,3,3)
    # Iterations, width of object, height of bin in base frame (In meters)
    #testmeasuredepth(20,0.047,0.0176,"Alberto Balsam", False)
    plot_bar([2,2,3,3,3,3,4,5,5,5],"Pressure test: Nivea Texture")
    #pressuretest(-0.1,10)
    #binplanecalibration(0.015)


def main():
    try:
        runstuff()
    except KeyboardInterrupt:
        return


if __name__ == '__main__':

    pubSuc = rospy.Publisher('/vacuum/set_suction_on', Bool, queue_size=1) # Talk to the festo ovem suction cup
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