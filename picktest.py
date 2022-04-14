#!/usr/bin/env python3  
import time
from typing import Container
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
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
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

def cartesian_traject(pose_goal):

    waypoints = []
    waypoints.append(copy.deepcopy(pose_goal))  
    (plan, fraction) = group.compute_cartesian_path(
                                                          waypoints,   # waypoints to follow
                                                          0.0005,        # eef_step
                                                          0         # jump_threshold
    )

    plan = group.retime_trajectory(robot.get_current_state(),plan,0.5)
    group.execute(plan)

    rospy.sleep(0.5)

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


def runstuff():

    # Recieving message from the camera
    #msg = rospy.wait_for_message("/pandaposition", PoseStamped)

    #while(msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0 ):
    #    msg = rospy.wait_for_message("/pandaposition", PoseStamped)


    
    #print(msg)
    input("Press to continue")
    (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))

    q = quaternion_from_euler(math.pi, 0, 0)

    #waypoints = []

    #print(f'{q[0]}, {q[1]}, {q[2]}, {q[3]}')

    # Setting initial pose 
    #position = msg.pose.position
    group.set_start_state_to_current_state()
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = rot[0]
    pose_goal.orientation.y = rot[1]
    pose_goal.orientation.z = rot[2]
    pose_goal.orientation.w = rot[3]
    #pose_goal.position.x = position.x
    pose_goal.position.x = trans[0]
    #pose_goal.position.y = position.y
    pose_goal.position.y = trans[1]
    pose_goal.position.z = trans[2]

    totdistance = 0
    # Vectors to collect force/distance data
    force = []
    distance = []
    xvec = []
    yvec = []
    waypoints = []
    realdist = []

    #Setting initial values of force and distance vectors
    forceval = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
    initpos = pose_goal.position.z
    initforce = forceval.wrench.force.z
    increment = 0.001

    while(totdistance < 0.02):
        pose_goal.position.z -= increment
        cartesian_traject(pose_goal)
        
        #input("press to increment down")

        forceval = rospy.wait_for_message("/franka_state_controller/F_ext", geometry_msgs.msg.WrenchStamped)
        force.append(abs(forceval.wrench.force.z - initforce))
        totdistance += increment 
        distance.append(totdistance)
        (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
        xvec.append(trans[0])
        yvec.append(trans[1])
        realdist.append(initpos-pose_goal.position.z)
        print(f'Force: {forceval.wrench.force.z - initforce} - Distance: {totdistance}')
        print(f'{forceval.wrench.force.z}')
        input("Press to continue")

    print("Finished")

    distance = np.asarray(distance)
    force = np.asarray(force)

    A = np.vstack([distance, np.ones(len(distance))]).T

    m, c = np.linalg.lstsq(A, force, rcond=None)[0]

    R = np.corrcoef(distance,force)

    _ = plt.plot(distance, force, 'o', label='Force measurements', markersize=10)
    _ = plt.plot(distance, m*distance + c, 'r', label='Fitted line')
    _ = plt.xlabel('Suction cup compression (m)')
    _ = plt.ylabel('Contact force (N)')
    _ = plt.legend()
    plt.show()

    pointval = m*0.008+c

    xvec = np.asarray(xvec)
    yvec = np.asarray(yvec)

    print(xvec-xvec[0])
    print(yvec-yvec[0])
    print(realdist)
    print(R)
    print(pointval)
    print(f'm: {m} - c: {c}')


    # group.set_pose_target(pose_goal)
    # plan1 = group.plan()
    # input("Showing planned path")
    # group.go(wait=True)

    # pose_goal.position.z -= 0.05
    # group.set_pose_target(pose_goal)
    # plan1 = group.plan()
    # input("Showing planned path")
    # group.go(wait=True)Framkvæmdastjóri Barnaverndar Reykjavíkur fór í leyfi

    # (plan, fraction) = group.compute_cartesian_path(
    #                                                      waypoints,   # waypoints to follow
    #                                                      0.05,        # eef_step
    #                                                      0         # jump_threshold
    # )
    # #group.set_pose_target(pose_goal)
    # #group.plan()
    # input("Viewing planned path - press to continue")
    # group.clear_pose_targets()
    # group.set_max_velocity_scaling_factor(0.05)
    # group.execute(plan)
    # print("finished moving")


    

    # input("Press to lower the suction cup")

    # currpose = group.get_current_pose()
    # print(f'Before: {currpose}')

    # for i in range(20):
    #     waypoints = []
    #     currpose=group.get_current_pose().pose
        
    #     wpose = geometry_msgs.msg.Pose()
    #     wpose.orientation = currpose.orientation
    #     wpose.position.y = currpose.position.y
    #     wpose.position.z = currpose.position.z - 0.002
    #     wpose.position.x = currpose.position.x

    #     waypoints.append(copy.deepcopy(wpose))

    #     #wpose.position.z = waypoints[0].position.z - 0.11

    #     #waypoints.append(copy.deepcopy(wpose))

    #     (plan, fraction) = group.compute_cartesian_path(
    #                                                         waypoints,   # waypoints to follow
    #                                                         0.001,        # eef_step
    #                                                         0.0)         # jump_threshold

    #     group.clear_pose_targets()
    #     group.set_max_velocity_scaling_factor(0.05)
    #     group.execute(plan)


    # currtime = rospy.Time.now()
    # currpose = group.get_current_pose()
    # currzpos = currpose.pose.position.z
    # print(f'After: {currpose}')


    #print(plan)
    # increment=0.0015

    # control = 1

    # while(control != '0'):
    #     group.shift_pose_target(2,-0.02)
    #     #group.set_pose_target(pose_goal)
    #     plan1 = group.plan()
    #     group.go(wait=True)
    #     group.clear_pose_targets()
    #     control = input("Enter 0 to break the loop \n")
    #     print(control)

def main():
    try:
        runstuff()
    except KeyboardInterrupt:
        return


if __name__ == '__main__':

    #rostopic pub  std_msgs/Bool true --once
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

    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10) # Talks to the moveit motion planner
    homePos()

    main()

    #while not rospy.is_shutdown():
    #    currstate = stateMachine(currstate)