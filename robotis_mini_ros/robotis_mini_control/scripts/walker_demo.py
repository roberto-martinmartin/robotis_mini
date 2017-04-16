#!/usr/bin/env python

import rospy
from robotis_mini_control.robotis_mini import RobotisMiniControlInterface
from std_srvs.srv import *
import time
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
import std_msgs.msg
import argparse
import std_srvs.srv

if __name__=="__main__":
    
    rospy.loginfo( "Interfacing and commanding to walk")
    
    parser = argparse.ArgumentParser(description='Walker demo')
    parser.add_argument('--real',action='store_true', help='define when using the real robot')
    
    options, args = parser.parse_known_args()
    
    if options.real:
        rospy.loginfo("Real Robot!")
    else:
        rospy.loginfo("Simulated Robot!")    
        rospy.wait_for_service('/gazebo/unpause_physics')
        unpause_simulation_srv = rospy.ServiceProxy('/gazebo/unpause_physics', std_srvs.srv.Empty)
        unpause_simulation_srv()
        rospy.loginfo( "Unpaused gazebo")
        time.sleep(1)

    rospy.loginfo( "Resetting robot and simulation")
    restart_srv = rospy.Publisher('/robotis_mini/restart_srv', std_msgs.msg.Empty, queue_size=1)
    new_params_pub = rospy.Publisher('/robotis_mini/walking_params', Float64MultiArray, queue_size=1)
    command_vel = rospy.Publisher('/robotis_mini/cmd_vel', Twist, queue_size=1)
    
    time.sleep(1)
    
    #After advertising topics
    rospy.init_node("walker_demo")
    time.sleep(1)
    
    # Stop walker
    rospy.loginfo("Stopping walker")
    restart_srv.publish(std_msgs.msg.Empty())
    time.sleep(5)    
 
    rospy.loginfo("RobotisMini Walker Demo Starting")
    
    msg=Twist()
    msg.linear.x=0
    msg.linear.y=0
    msg.angular.z=0
    command_vel.publish(msg)
    
    #robotis_mini.set_walk_velocity(0,0,0)

    #robotis_mini.set_walk_velocity(0.2,0,0)
    #rospy.sleep(3)
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
