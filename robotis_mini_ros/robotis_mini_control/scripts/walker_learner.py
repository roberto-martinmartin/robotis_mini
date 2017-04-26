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
from gazebo_msgs.msg import LinkStates
import numpy as np
from itertools import product
import rospkg
import pickle

import tf

class WalkerLearner:
    
    def __init__(self, real_robot=False):
        
        self.real_robot = real_robot
        if self.real_robot:
            rospy.loginfo("Real Robot!")
        else:
            rospy.loginfo("Simulated Robot!")    
            rospy.wait_for_service('/gazebo/unpause_physics')
            self.unpause_simulation_srv = rospy.ServiceProxy('/gazebo/unpause_physics', std_srvs.srv.Empty)
            
            self.head_sub=rospy.Subscriber("/gazebo/link_states", LinkStates, self.head_state_cb, queue_size=1)
        
        self.restart_srv = rospy.Publisher('/robotis_mini/restart_srv', std_msgs.msg.Empty, queue_size=1)
        self.new_params_pub = rospy.Publisher('/robotis_mini/walking_params', Float64MultiArray, queue_size=1)
        self.command_vel = rospy.Publisher('/robotis_mini/cmd_vel', Twist, queue_size=1)

        self.cycle_period_limits = [2, 14]
        self.amplitude_limits = [-0.6,0.6]
        self.amplitude_offset_limits = [-1, 1]
        self.phase_limits = [-0.2, 0.2]

        self.final_state = dict()
        self.reward = dict()

        self.tf_pub = tf.TransformBroadcaster()

        # Reset the simulation and the robot
        self.unpause_simulation_srv()
        rospy.loginfo("Unpaused gazebo")
        time.sleep(3)

    def head_state_cb(self, link_states_msg):
        #process the message
        base_link_index = link_states_msg.name.index("robotis_mini::base_link")
        self.current_baselink_pose = link_states_msg.pose[base_link_index]

        # Visualization in RVIZ
        try:
            current_time = rospy.Time.now()
            self.tf_pub.sendTransform((self.current_baselink_pose.position.x,
                                       self.current_baselink_pose.position.y,
                                       self.current_baselink_pose.position.z),
                                      (self.current_baselink_pose.orientation.x,
                                       self.current_baselink_pose.orientation.y,
                                       self.current_baselink_pose.orientation.z,
                                       self.current_baselink_pose.orientation.w),
                                      current_time,
                                      'frame_reward',
                                      "world")
        except:
            x = 1 #do nothing

        
    def start(self):
        
        for search_steps in range(2):

            # Generate new params
            # [cycle, foot, ankle, knee, thigh, hip]
            # [amplitude, amplitude_offset, phase_offset]
            new_params = [np.random.uniform(self.cycle_period_limits[0], self.cycle_period_limits[1]),
                          # foot
                          np.random.uniform(self.amplitude_limits[0], self.amplitude_limits[1]),
                          #np.random.uniform(self.amplitude_offset_limits[0], self.amplitude_offset_limits[1]),
                          0,
                          np.random.uniform(self.phase_limits[0], self.phase_limits[1]),
                          # ankle
                          np.random.uniform(self.amplitude_limits[0], self.amplitude_limits[1]),
                          np.random.uniform(self.amplitude_offset_limits[0], self.amplitude_offset_limits[1]),
                          np.random.uniform(self.phase_limits[0], self.phase_limits[1]),
                          # knee
                          np.random.uniform(self.amplitude_limits[0], self.amplitude_limits[1]),
                          np.random.uniform(self.amplitude_offset_limits[0], self.amplitude_offset_limits[1]),
                          np.random.uniform(self.phase_limits[0], self.phase_limits[1]),
                          # thigh
                          np.random.uniform(self.amplitude_limits[0], self.amplitude_limits[1]),
                          np.random.uniform(self.amplitude_offset_limits[0], self.amplitude_offset_limits[1]),
                          np.random.uniform(self.phase_limits[0], self.phase_limits[1]),
                          # hip
                          np.random.uniform(self.amplitude_limits[0], self.amplitude_limits[1]),
                          #np.random.uniform(self.amplitude_offset_limits[0], self.amplitude_offset_limits[1]),
                          0,
                          np.random.uniform(self.phase_limits[0], self.phase_limits[1])
                          ]

            print "Sending params ", new_params

            # Send the new params
            new_params_msg = Float64MultiArray()
            new_params_msg.data = new_params
            self.new_params_pub.publish(new_params_msg)

            rospy.loginfo("Resetting robot and simulation")
            rospy.loginfo("Stopping walker")
            self.restart_srv.publish(std_msgs.msg.Empty())
            time.sleep(6)
            
            # Send the walking command     
            msg=Twist()
            msg.linear.x=0
            msg.linear.y=0
            msg.angular.z=0
            self.command_vel.publish(msg)
            
            # Wait for the robot to walk
            time.sleep(20)
            
            # Collect the last state
            self.final_state[str(new_params)] = self.current_baselink_pose
            self.reward[str(new_params)] = [0.0, self.current_baselink_pose.position.z][self.current_baselink_pose.position.z>0.05]

        rospy.loginfo("Stopping walker end")
        self.restart_srv.publish(std_msgs.msg.Empty())
        time.sleep(6)

        timestr = time.strftime("%Y%m%d-%H%M%S")
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        pathstr = rospack.get_path('robotis_mini_control')
        pickle.dump(self.final_state, open(pathstr + '/data/finalstate_' + timestr + '.pkl', "wb"))
        pickle.dump(self.reward, open(pathstr + '/data/reward_' + timestr + '.pkl', "wb"))

        print "Final rewards"
        ep_ctr = 1
        for key, value in self.reward.iteritems():
            print "Episode " + str(ep_ctr) + " reward " + str(value)
            ep_ctr += 1


if __name__=="__main__":
    
    rospy.loginfo( "Learn optimal walking parameters")
    
    parser = argparse.ArgumentParser(description='Walker demo')
    parser.add_argument('--real',action='store_true', help='define when using the real robot')
    
    options, args = parser.parse_known_args()
          
    walker_learner = WalkerLearner(options.real)

    #After advertising topics
    rospy.init_node("walker_learner")

    rospy.loginfo("RobotisMini Walker Learner Starting")
    
    walker_learner.start()
        
    rospy.loginfo("RobotisMini Walker Learner Finished")
