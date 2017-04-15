#!/usr/bin/env python

import rospy
from robotis_mini_gazebo.robotis_mini import RobotisMiniControlInterface
from std_srvs.srv import *
import time
from std_msgs.msg import Float64
import std_msgs.msg
    

if __name__=="__main__":
    
    print "Start the demo"
    
    print "Resetting robot and simulation"
    
    sw = rospy.Publisher('/robotis_mini/stop_srv', std_msgs.msg.Empty, queue_size=1)
    
    if False:
        
        # Setup the publishers for each joint
        p1 = rospy.Publisher('/robotis_mini/l_shoulder_joint_position_controller/command', Float64, queue_size=1)
        p2 = rospy.Publisher('/robotis_mini/r_shoulder_joint_position_controller/command', Float64, queue_size=1)
        p3 = rospy.Publisher('/robotis_mini/l_biceps_joint_position_controller/command', Float64, queue_size=1)
        p4 = rospy.Publisher('/robotis_mini/r_biceps_joint_position_controller/command', Float64, queue_size=1)
        p5 = rospy.Publisher('/robotis_mini/l_elbow_joint_position_controller/command', Float64, queue_size=1)
        p6 = rospy.Publisher('/robotis_mini/r_elbow_joint_position_controller/command', Float64, queue_size=1)
        p7 = rospy.Publisher('/robotis_mini/l_hip_joint_position_controller/command', Float64, queue_size=1)
        p8 = rospy.Publisher('/robotis_mini/r_hip_joint_position_controller/command', Float64, queue_size=1)
        p9 = rospy.Publisher('/robotis_mini/l_thigh_joint_position_controller/command', Float64, queue_size=1)
        p10 = rospy.Publisher('/robotis_mini/r_thigh_joint_position_controller/command', Float64, queue_size=1)
        p11 = rospy.Publisher('/robotis_mini/l_knee_joint_position_controller/command', Float64, queue_size=1)
        p12 = rospy.Publisher('/robotis_mini/l_ankle_joint_position_controller/command', Float64, queue_size=1)
        p13 = rospy.Publisher('/robotis_mini/r_knee_joint_position_controller/command', Float64, queue_size=1)
        p14 = rospy.Publisher('/robotis_mini/r_ankle_joint_position_controller/command', Float64, queue_size=1)
        p15 = rospy.Publisher('/robotis_mini/l_foot_joint_position_controller/command', Float64, queue_size=1)
        p16 = rospy.Publisher('/robotis_mini/r_foot_joint_position_controller/command', Float64, queue_size=1)
        time.sleep(1)
    
    rospy.init_node("walker_demo")
    
    time.sleep(1)
    
    # Stop walker
    sw.publish(std_msgs.msg.Empty())
    
    time.sleep(5)
    print "Stopping walker"
    
    if False:
        # Set configuration
        p1.publish(0.0)
        p2.publish(0.0)
        p3.publish(0.0)
        p4.publish(0.0)
        p5.publish(0.0)
        p6.publish(0.0)
        p7.publish(0.0)
        p8.publish(0.0)
        p9.publish(0.0)
        p10.publish(0.0)
        p11.publish(0.0)
        p12.publish(0.0)
        p13.publish(0.0)
        p14.publish(0.0)
        p15.publish(0.0)
        p16.publish(0.0)
        
        time.sleep(2.0)
        print "Resetting robot"
        
        rospy.wait_for_service('/gazebo/pause_physics')
        rospy.wait_for_service('/gazebo/reset_world')
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            pause_simulation = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            pause_simulation()
            print "pause_physics Service call succeeded"
            time.sleep(1)
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()
            print "reset_world Service call succeeded"
            time.sleep(1)
            unpause_simulation = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            unpause_simulation()
            print "unpause_simulation Service call succeeded"
            time.sleep(1)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    rospy.loginfo("Instantiating RobotisMini Controller Interface")
    robotis_mini=RobotisMiniControlInterface(real_robot=True)
    time.sleep(1)
 
    rospy.loginfo("RobotisMini Walker Demo Starting")


    #robotis_mini.set_walk_velocity(0.2,0,0)
    #rospy.sleep(3)
    robotis_mini.set_walk_velocity(0,0,0)
    #rospy.sleep(10000)
    #robotis_mini.set_walk_velocity(0,1,0)
    #rospy.sleep(3)
    #robotis_mini.set_walk_velocity(0,-1,0)
    #rospy.sleep(3)
    #robotis_mini.set_walk_velocity(-1,0,0)
    #rospy.sleep(3)
    #robotis_mini.set_walk_velocity(1,1,0)
    #rospy.sleep(5)
    #robotis_mini.set_walk_velocity(0,0,0)
        
    rospy.loginfo("RobotisMini Walker Demo Finished")
