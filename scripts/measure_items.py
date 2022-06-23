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

# Mininum and maximum points of the box in world coordinates - Given that box has not been moved

YMIN = -0.1139
YMAX = 0.1838
XMIN = 0.33
XMAX = 0.5
ZMIN = -0.016
ZMAX = 0.2

# Instantiate CvBridge
bridge = CvBridge()

image_color = "/camera/color/image_raw"
image_depth = "/camera/aligned_depth_to_color/image_raw"
#folder_path = "/mnt/bigdata/eidur14/data"
#folder_path = "/home/lab/Pictures/data/eidur14"
folder_path = "/home/lab/Documents/Eidur/Data/newdata"
#folder_path = "/mnt/bigdata/eidur14"

counter = 0

## Helper functions ##

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

# Checks that the pose target coordinate lies within the box boundaries
def iswithin(pose_target):
    # Booleans for each dimension
    xvalid = pose_target.position.x > XMIN and pose_target.position.x < XMAX
    yvalid = pose_target.position.y > YMIN and pose_target.position.y < YMAX
    zvalid = pose_target.position.z > ZMIN and pose_target.position.z < ZMAX

    return xvalid and yvalid and zvalid

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

# Takes in a stamped pose and returns an unstamped one
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

# Takes in coordinates from camera and transforms to base link frame
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

    # Point transformed from camera to world frame
    p = listener.transformPoint("panda_link0",camframePt)   
    
    # Pose created for the end effector in the world frame, x-y orientation parallel
    # to bin plane (And world frame)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = 1
    pose_target.orientation.y = 0
    pose_target.orientation.z = 0
    pose_target.orientation.w = 0
    pose_target.position.x = p.point.x
    pose_target.position.y = p.point.y
    # Position 3 cm above target pick point
    pose_target.position.z = p.point.z+0.015
    currentbox = [quat.x, quat.y, quat.z, quat.w]
    #addMarker(pose_target,"panda_link0", False)
    #print(currentbox)
    # The transformed pose, current bounding box (Transferred through quaternion from vision node)
    # and the reported depth of the object by the camera returned
    return pose_target, quat, camframePt.point.x    
    

## Main functions ##

def homePos():
    # print("Moving to home position")
    # Home position using joint values
    # group.clear_pose_targets()
    # group_variable_values = group.get_current_joint_values()
    # group_variable_values[0] = float(radians(0))
    # group_variable_values[1] = float(radians(-25))
    # group_variable_values[2] = float(radians(1))
    # group_variable_values[3] = float(radians(-133))
    # group_variable_values[4] = float(radians(0))
    # group_variable_values[5] = float(radians(109))
    # group_variable_values[6] = float(radians(45)) # 45 is straight
    # group.set_joint_value_target(group_variable_values)

    # Joint positions in radians (L515)
    # group_variable_values = group.get_current_joint_values()
    # group_variable_values[0] = 0.039827141041538304
    # group_variable_values[1] = -0.6856789446295354
    # group_variable_values[2] = -0.051094866318138026
    # group_variable_values[3] = -2.6372590758340397
    # group_variable_values[4] = -0.022729468238022594
    # group_variable_values[5] = 1.9992456805838477
    # group_variable_values[6] = 0.7680859528355461 # 
    # group.set_joint_value_target(group_variable_values)

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
    
    plan1 = group.plan()
    group.go(wait=True)
    group.clear_pose_targets()

# Moves down vertically towards a pick point, takes in a force goal as a stopping condition indicating the pick point has been reached
# Depricated
def movetotouch(initial_pose, force_goal,debug=True):
    
    totdistance = 0
    pick=False

    # Initializing current position for the loop
    (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))

    #Setting initial values of force and distance vectors
    forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
    initpos = initial_pose.position.z
    initforce = forcemsg.wrench.force.z
    increment = 0.001
    forceval=0

    while(trans[2] > 0.02 and forceval < force_goal):
        initial_pose.position.z -= increment
        cartesian_traject(initial_pose,0.0005,0.5,True)
        
        #input("press to increment down")

        forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
        forceval=float(abs(forcemsg.wrench.force.z - initforce))
        # force.append(forceval)
        totdistance += increment 
        (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
        # xvec.append(trans[0])
        # yvec.append(trans[1])
        if(debug):
            print(f'Force: {forceval} - Distance: {totdistance}')
            #input("Press to continue")

    pubSuc.publish(Bool(True))
    rospy.sleep(0.3)
    pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
    print(f'Pressure: {pressure.data}')
    
    if(pressure.data <= -0.1):
        pick=True

    currentp=group.get_current_pose()
    currentp=pose_unstamped(currentp)

    return pick, currentp

# Depth measurements using force feedback from Panda
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

    #Setting initial values of force and distance vectors
    rospy.sleep(1)
    forcemsg = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
    forceval=0
    initpos = pose_goal.position.z
    initforce = forcemsg.wrench.force.z
    increment = 0.001

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
        print(f'Force: {forceval}- Distance: {totdistance}')

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

# Uses pressure to find pickpoint
# Returns estimated depth at surface, depth at vacuum pick and if the pick was successfull
def measuredepthP(camera_initial_height, refdist):

    # Getting current pose of the robot
    currentp=group.get_current_pose()
    currentp=pose_unstamped(currentp)

    # Pose goal at the object
    pose_goal=currentp

    # Move towards object/surface
    #cartesian_traject(pose_goal,0.05,0.5,True)

    #input("Review position - Press to continue")

    totdistance = 0
    # Vectors to collect force/distance da        forceval = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)

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

    # Drives down while force pressure has not been reached. Maximum approach distance 2,5 cm
    while(totdistance < 0.055 and pressure.data > -0.1):
        pose_goal.position.z -= increment
        # waypoint,step,vel_scale=0.5,pause=False,sleeptime=0.6
        cartesian_traject(pose_goal,0.0005,0.1)
        #input("press to increment down")
        rospy.sleep(0.05)
        # Force value on the end effector acquired
        pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
        totdistance += increment 
        #(trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
        print(f'Pressure: {pressure.data}- Distance: {totdistance}')
        

    print("Finished measuring")

    # If the loop was broken by trigger pressure
    if pressure.data < -0.1:
        # If the loop was entered, else the pick point was exceeded
        # and the measurement is discarded since the desired value is 
        # the lowest depth value at which suction grasp is achieved.
        if totdistance > 0:
            pick=True
        else:
            pick=False
    # If loop is broken by distance, the depth measurment is discarded (pick=False by default)
    else:
        pubSuc.publish(Bool(False))
        
    # Getting the position of the suction end in world coordinates
    sucendPt=PointStamped()
    sucendPt.header.frame_id = "panda_suction_end"
    grndtruthpos = listener.transformPoint("panda_link0",sucendPt)
    grndtruthheight=grndtruthpos.point.z

    # For sanity check, give a rough estimatin on the height of the object:
    # height at pick - height at bottom of bin
    print(f'Height of object: {grndtruthheight-0.0176}')

    #Calculating depth at surface and depth at pick point
    grndtruthdepth=camera_initial_height-(grndtruthheight+refdist)
    grndtrthpick=camera_initial_height-grndtruthheight

    return grndtruthdepth, grndtrthpick, pick

# Moves toward the pick point, performs measurement and gathers images and labels
def moveAbove(pose_target, camdepth):
    global currbox
    captureImage()
    # Getting the position of the camera in world coordinates
    camPt=PointStamped()
    camPt.header.frame_id = "camera_color_frame"
    camInitial = listener.transformPoint("panda_link0",camPt)
    initialheight=camInitial.point.z

    print(f'Initial height of camera: {initialheight}')

    # Move the suction cup slightly above the object
    cartesian_traject(pose_target,0.05,0.8)
    
    # Moves vertically towards target pick point until a pick is achieved
    measured_depth,measuredpp,pick= measuredepthP(initialheight,0.003)
    #measured_depth=initialheight-currentp.position.z+0.004

    print(f'Depth found: {measured_depth} - Vision depth: {camdepth} -Press any key to continue')

    # Labels only gathered if pick is considered a success.
    if(pick):
        datawrite(currbox,measuredpp,camdepth)

    return pick

# Moving items within the bin, random coordinates chosen within bin with a slight random rotation
# Safemode used to verify that the poses are valid, observing markers in Rviz
def shuffleobjects(safemode=True): 

    # Getting current pose and transforming it to non-stamped pose
    currentp=group.get_current_pose()
    currentp=pose_unstamped(currentp)

    # Go up 3 cm
    currentp.position.z += 0.03

    if(safemode):
        addMarker(currentp, "panda_link0")

        input("Review marker - Press to continue")

    #cartesian_traject(currentp, 0.05,0.5,pause=True)

    # Add a random orientation
    q = quaternion_from_euler(math.pi, 0, np.random.uniform(0, math.pi/8))

    #x = currentp.position.x
    # Try to keep object around middle
    x = np.random.uniform(XMIN+0.01, XMAX-0.06)
    #x = 0.39
    y = np.random.uniform(YMIN+0.06, YMAX-0.06)
    z = currentp.position.z

    pose_target=createpose(x,y,z,q[0],q[1],q[2],q[3])

    if(safemode):
        addMarker(pose_target, "panda_link0")
        input("Review marker - Press to continue")
    
    cartesian_traject(pose_target, 0.05,0.5,pause=True)
    # Shut of vacuum
    pubSuc.publish(Bool(False))

def captureImage():
    global counter
    colormsg = rospy.wait_for_message(image_color, Image)
    depthmsg = rospy.wait_for_message(image_depth, Image)
    color_frame = bridge.imgmsg_to_cv2(colormsg, "bgr8")
    depth_frame = bridge.imgmsg_to_cv2(depthmsg, desired_encoding="passthrough")
    # To visualize the depth images
    # depth_image = np.array(depth_frame, dtype=np.float32)
    # depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image), cv2.COLORMAP_RAINBOW)
    imgname = str(counter).zfill(4)
    cv2.imwrite(os.path.join(dirName, imgname+"color.png"), color_frame)
    cv2.imwrite(os.path.join(dirName, imgname+"depth.png"), depth_frame)

# Writes out data of bounding box as well as calculated depth
# And the height of the end effector at pick, as the raw depth
def datawrite(currentbox,depth,camdepth):
    global counter
    imgname = str(counter).zfill(4)
    # Label format: x left bottom, y left bottom, width, height, measured depth, camera depth
    # The label format would ideally use x middle, y middle but a large dataset has been gathered
    # Using this format, the format will be kept for consistency
    f= open(os.path.join(dirName,imgname+".txt"),"w+")
    f.write("0 %.9f %.9f %.9f %.9f %.6f %.6f\r\n" % (float(currentbox.x),float(currentbox.y),float(currentbox.z),float(currentbox.w),float(depth),float(camdepth)))
    f.close()
    #print("0 %.9f %.9f %.9f %.9f %.6f %.6f\r\n" % (float(currentbox.x),float(currentbox.y),float(currentbox.z),float(currentbox.w),float(depth),float(camdepth)))
    counter += 1
 
# Runs in an infinite loop gathering depth measurements, to break simply press ctrl+c.
def stateMachine(currState):
    global endPosition, start, currbox, dirName, camdepth
    #The first state starts by creating a new directory to save the images.
    if currState == 0: 
        folder = ""
        print("Create new dir")
        folder = input("Folder name\n") # user creates a folder
        dirName = folder_path + "/" + folder
        try:
            # Create target Directory
            os.mkdir(dirName)
            print("Directory " , dirName ,  " Created ") 
            counter = 0
        except:
            print("Directory " , dirName ,  " already exists") 
        return 1
    elif currState == 1: 
        # The second state waits for a message from the find_pickpoint.py through a ROS topic where it gets an object position from the camera, 
        # and then the TF listener transforms the object position in camera coordinates to world coordinates.
        rospy.sleep(1)
        currPoint=PointStamped()
        currPoint.header.frame_id = "panda_suction_end"
        home = listener.transformPoint("panda_link0",currPoint)
        
        try:
            # Getting current position and rotation of current position of panda_suction_end
            #(trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time())
            # Setting end effector parallel to the bin
            #q = quaternion_from_euler(math.pi, 0, 0)
            # Wait for pose goal from th vision side
            msg = rospy.wait_for_message("/pickpoint", PoseStamped)
            rospy.loginfo("Received at goal message!")
            addMarker(msg.pose, "camera_color_frame")
            #input("Review position of marker")
            endPosition, currbox, camdepth = endPos(msg) # transforming from camera frame to world frame

            if (not iswithin(endPosition)):
                print(f'({endPosition.position.x},{endPosition.position.y},{endPosition.position.z}) not within bounds!')
                return 1
            else: 
                #emptycount = 0
                return 2
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("look up fail")    

    elif currState == 2:
        # At the third state, the actual measurement is performed. The brings the suction cup
        # sligthly above the pick point detected by the camera to account for possible depth
        # underestimation from camera. The end effector is then slowly driven towards the object 
        # Until a certain approach length is excceded, which indicates a non-succsessful pick, or
        # when certain pressure drop is achieved within the vacuum system which indicates a succsessful
        # pick. If a pick is achieved the object is relocated to a random position within the bin with 
        # and reorientated. If not, the robot goes into the home position and begins again.
        pick=moveAbove(endPosition,camdepth)
        if pick == False:
            pubSuc.publish(bool(False))
            homePos()
            return 1
        else:
            #rospy.sleep(0.2)
            return 3
        
    elif currState == 3:
        # The fourth state performs the relocation of the object and then moves to the robot
        # to the home position. Execution is then halted for two seconds two ensure that the
        # object is stationary before accepting pose goals from the vision side. Which might
        # need less or more time depending on the object.
        shuffleobjects(False)
        homePos()
        rospy.sleep(4)
        return 1
               
if __name__ == '__main__':
    emptycount = 0
    #rostopic pub  std_msgs/Bool true --once
    pubSuc = rospy.Publisher('/vacuum/set_suction_on', Bool, queue_size=1) # Talk to the festo ovem suction cup
    rospy.init_node('measure_items')
    listener = tf.TransformListener()
    markpub = rospy.Publisher("posmarker", Marker, queue_size=1)
    # Movegroup classes started
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("panda_arm")
    group.allow_replanning(True)
    group.set_planning_time(1)
    # Information on end effector link and tolerances
    ref=group.get_pose_reference_frame()
    eef_link=group.get_end_effector_link()
    orient = group.get_goal_orientation_tolerance()
    goal_tol=group.get_goal_tolerance()
    # print(f'Reference frame: {ref}')
    # print(f'End effector link: {eef_link}')
    # print(f'Goal tolerance: {goal_tol}')
    # print(f'Orientation tolerance: {orient}')
    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10) # Talks to the moveit motion planner
    homePos()
    empty = 0
    now = datetime.now()
    # dd/mm/YY H:M:S
    dt_string = now.strftime("%d-%m-%Y-%H:%M")
    if empty == 0:
        currstate = stateMachine(empty)

    while not rospy.is_shutdown():
        currstate = stateMachine(currstate)
        
            

