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

XMAX = 0.36 # 0.38
XMAX2 = 0.405
XMIN = 0.28
YMAX = 0.137
YMIN = -0.096
#Home position
HOMEX = 0.3492#0.3091
HOMEY = 0.0005#0.0048
HOMEZ = 0.4460#0.4234

locerror = 0.01 # max error in movement
moveit_commander.roscpp_initialize(sys.argv)
# Instantiate CvBridge
bridge = CvBridge()

image_color = "/camera/color/image_raw"
folder_path = "/home/lab/Pictures/data"
counter = 0
error = 0.05 # error from the camera, since it is tilted
suctionCup = 0.12#0.045 #decreas if want to go further down 0.034 length from endeffector to end of suction cup red suction 5.5cm
dropSpace = 0.015

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
        rospy.sleep(1)

def homePos():
    print("Moving to home position")
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    #group_variable_values = group.get_current_joint_values()
    #group_variable_values[0] = float(radians(0))
    #group_variable_values[1] = float(radians(-25))
    #group_variable_values[2] = float(radians(1))
    #group_variable_values[3] = float(radians(-133))
    #group_variable_values[4] = float(radians(0))
    #group_variable_values[5] = float(radians(109))
    #group_variable_values[6] = float(radians(45)) # 45 is straight
    #group.set_joint_value_target(group_variable_values)

    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = 0
    group_variable_values[1] = -0.6999209085682471
    group_variable_values[2] = 0.015764035018303778
    group_variable_values[3] = -2.479506837022749
    group_variable_values[4] = -0.03019742977288034
    group_variable_values[5] = 1.8284102745983333
    group_variable_values[6] = 0.7890444130053123
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


def runstuff():

    # Getting position of objects
    #msg = rospy.wait_for_message("/pandaposition", PoseStamped)
    #addMarker(msg.pose, "camera_color_frame")

    # Getting the position of the camera in world coordinates
    camPt=PointStamped()
    camPt.header.frame_id = "camera_color_frame"
    camInitial = listener.transformPoint("panda_link0",camPt)
    initialheight=camInitial.point.z

    # Saving estimated depth from L515 camera
    #lidardepth=msg.pose.position.x

    print(f'Initial height of camera: {initialheight}')

    #q = quaternion_from_euler(math.pi, 0, 0)

    # Point transformed to world frame - Middle point
    #endPosition, currbox, camdepth = endPos(msg,q)
    # pose_targetM = geometry_msgs.msg.Pose()
    # pose_targetM.orientation.x = 1
    # pose_targetM.orientation.y = 0
    # pose_targetM.orientation.z = 0
    # pose_targetM.orientation.w = 0
    # pose_targetM.position.x = 0.41655
    # pose_targetM.position.y = 0
    # pose_targetM.position.z = 0.0196
    # pose_goalM=pose_targetM

    # Point transformed to world frame - Further left
    #endPosition, currbox, camdepth = endPos(msg,q)
    # pose_targetFL = geometry_msgs.msg.Pose()
    # pose_targetFL.orientation.x = 1
    # pose_targetFL.orientation.y = 0
    # pose_targetFL.orientation.z = 0
    # pose_targetFL.orientation.w = 0
    # pose_targetFL.position.x = 0.49758
    # pose_targetFL.position.y = 0.15024
    # pose_targetFL.position.z = 0.0207
    # pose_goalFL=pose_targetFL

    # Point transformed to world frame - Further right
    #endPosition, currbox, camdepth = endPos(msg,q)
    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.x = 1
    # pose_target.orientation.y = 0
    # pose_target.orientation.z = 0
    # pose_target.orientation.w = 0
    # pose_target.position.x = 0.51112
    # pose_target.position.y = -0.13277
    # pose_target.position.z = 0.02378
    # pose_goal=pose_target

    # Point transformed to world frame - Near right
    #endPosition, currbox, camdepth = endPos(msg,q)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = 1
    pose_target.orientation.y = 0
    pose_target.orientation.z = 0
    pose_target.orientation.w = 0
    pose_target.position.x = 0.3939
    pose_target.position.y = -0.112898
    pose_target.position.z = 0.0276
    pose_goal=pose_target

    # Point transformed to world frame - Near left
    #endPosition, currbox, camdepth = endPos(msg,q)
    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.x = 1
    # pose_target.orientation.y = 0
    # pose_target.orientation.z = 0
    # pose_target.orientation.w = 0
    # pose_target.position.x = 0.38055
    # pose_target.position.y = 0.16
    # pose_target.position.z = 0.0222
    # pose_goal=pose_target

    addMarker(pose_goal, "panda_link0")

    input("Review goal - Press to continue")

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
    pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
    # Vacuum on
    #pubSuc.publish(Bool(True))

    while(totdistance < 0.05 and forceval < 0.5):
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

    print("Finished")

    # Getting the position of the suction end in world coordinates
    sucendPt=PointStamped()
    sucendPt.header.frame_id = "panda_suction_end"
    grndtruthpos = listener.transformPoint("panda_link0",sucendPt)
    grndtruthheight=grndtruthpos.point.z

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
    markpub = rospy.Publisher("posmarker", Marker, queue_size=1)

    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10) # Talks to the moveit motion planner
    homePos()

    main()

    #while not rospy.is_shutdown():
    #    currstate = stateMachine(currstate)