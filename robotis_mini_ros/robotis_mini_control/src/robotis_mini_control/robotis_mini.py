import random
from threading import Thread
import math
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist


class RobotisMiniControlInterface:
    """
    Client ROS class to receive control commands for Robotis Mini in python and interface with ROS controllers.
    The commands can be:
    - Desired joint space configurations -> republished in ROS as commands for the joint space controllers
    - Desired operational space displacements on the plane (walking command) -> republished in ROS as twist for the walker controller
    Implements also a simple interpolator between current and desired joint space configuration
    """
    
    def __init__(self,ns="/robotis_mini/", real_robot=False):
		
        self.ns=ns
        self.q_names=None	#Array with the names of each joint
        self.q_states=None	#Array with the state of each joint
        
        self.real_robot = real_robot    #To switch easily between simulated robot (Ros controllers + gazebo) and the real robot
        
        self._q_sub=rospy.Subscriber(ns+"joint_states", JointState, self._cb_q, queue_size=1)
        rospy.loginfo("Waiting for joints to be populated...")
        while not rospy.is_shutdown():
            if self.q_names is not None: break
            rospy.sleep(0.1)            
            rospy.loginfo("Waiting for joints to be populated...")
        rospy.loginfo("Joints populated")
        
        if not self.real_robot:
            rospy.loginfo("Creating publishers for the desired state (qd) of each joint")
            self._qd_pubs={}	#Dictionary with key=JointName and value=Publisher of desired joint state for that joint
            for j in self.q_names:
                p=rospy.Publisher(self.ns+j+"_position_controller/command", Float64, queue_size=1)
                self._qd_pubs[j]=p
        else:
            rospy.loginfo("Creating one publisher for the desired state (qd) of each joint")
            self._qd_pubs={}	#Dictionary with only one entry 'pub' to publisher
            p=rospy.Publisher(self.ns+"goal", JointState, queue_size=10)
            self._qd_pubs['pub']=p
        
        rospy.sleep(1)
        
        self._pub_cmd_vel=rospy.Publisher(ns+"cmd_vel", Twist, queue_size=1)
    
    ## Publishes a message (Twist) with the desired walking velocity
    def set_walk_velocity(self, x, y, theta):
        msg=Twist()
        msg.linear.x=x
        msg.linear.y=y
        msg.angular.z=theta
        self._pub_cmd_vel.publish(msg)
        
    ## Callback for the q (joint states) from ROS. q stored and is used to create interpolated trajectories
    def _cb_q(self, msg):
		# The first time we get the names of the joints
        if self.q_names is None:
            self.q_names=msg.name
        # We always update the state of the joints
        self.q_states=msg.position        
    
    ## Returns the latest received q (joint states) from ROS as a dictionary of (jointName, jointState)
    def get_q(self):
		# If we haven't received any message with the joint states yet, we return None
        if self.q_names is None or self.q_states is None: return None
        
        # Else, we return a dictionary with keys=jointNames and values=jointStates
        return dict(zip(self.q_names, self.q_states))

	## Publish desired joint state values for each joint
    def set_qd(self, qd_end_dict):
        if not self.real_robot:
            for jn,qd in qd_end_dict.items():
                if jn not in self.q_names:
                    rospy.logerror("Invalid joint name "+jn)
                    continue
                self._qd_pubs[jn].publish(qd)
        else:
            jsm = JointState()
            for jn,qd in qd_end_dict.items():
                if jn not in self.q_names:
                    rospy.logerror("Invalid joint name "+jn)
                    continue
                jsm.name.append(jn)
                jsm.position.append(qd)
            self._qd_pubs['pub'].publish(jsm)
	
	## Publish a sequence of desired joint state values for each joint interpolating between the current and the desired joint state
    def set_qd_interpolated(self, qd_end_dict, interpolation_time=2):
        q_init_dict=self.get_q()
        t_init=time.time()
        t_end=t_init+interpolation_time
        r=rospy.Rate(100)
        while not rospy.is_shutdown():
            t_curr=time.time()
            if t_curr>t_end: break
            ratio=(t_curr-t_init)/interpolation_time    
            qd_curr_dict=interpolate(qd_end_dict, q_init_dict, ratio)                        
            self.set_qd(qd_curr_dict)
            r.sleep()

## Create a dictionary with the linear interpolated values between q1 and q2 based on coefa (between 0->q2 and 1->q1)
def interpolate(q1_dict, q2_dict, coefa):
    q_dict={}
    q_names= q1_dict.keys()
    for jn in q_names:
        q_dict[jn]=q1_dict[jn]*coefa+q2_dict[jn]*(1-coefa)
    return q_dict
