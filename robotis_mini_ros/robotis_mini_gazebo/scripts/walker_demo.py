#!/usr/bin/env python

import rospy
from robotis_mini_gazebo.robotis_mini import RobotisMini


if __name__=="__main__":
    rospy.init_node("walker_demo")
    
    rospy.loginfo("Instantiating RobotisMini Client")
    robotis_mini=RobotisMini()
    rospy.sleep(1)
 
    rospy.loginfo("RobotisMini Walker Demo Starting")


    #robotis_mini.set_walk_velocity(0.2,0,0)
    #rospy.sleep(3)
    robotis_mini.set_walk_velocity(1,0,0)
    rospy.sleep(3)
    #robotis_mini.set_walk_velocity(0,1,0)
    #rospy.sleep(3)
    #robotis_mini.set_walk_velocity(0,-1,0)
    #rospy.sleep(3)
    #robotis_mini.set_walk_velocity(-1,0,0)
    #rospy.sleep(3)
    #robotis_mini.set_walk_velocity(1,1,0)
    #rospy.sleep(5)
    robotis_mini.set_walk_velocity(0,0,0)
        
    rospy.loginfo("RobotisMini Walker Demo Finished")
