#!/usr/bin/env python

from threading import Thread
import rospy
import math
from robotis_mini_control.robotis_mini import RobotisMiniControlInterface
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Empty, Float64, Float64MultiArray
import argparse
import std_srvs.srv
import sys


class SinusoidFunction:
    """
    SinusoidFunction for single joints CPG style
    Provides a parameterized sine wave function as y=amplitude_offset+amplitude*(phase_offset+angular_frequency*x)
    """
    def __init__(self):
        self.amplitude_offset=0
        self.amplitude=1
        self.phase_offset=0
        self.angular_frequency=1
        
    def get(self, x):
        """ x between 0 and 1"""
        f = math.sin(self.phase_offset + self.angular_frequency*x)
        return self.amplitude_offset + self.amplitude*f        
        
    def clone(self):
        z=SinusoidFunction()
        z.amplitude_offset=self.amplitude_offset
        z.amplitude=self.amplitude
        z.phase_offset=self.phase_offset
        z.angular_frequency=self.angular_frequency
        return z
    
    def mirror(self):
        z=self.clone()
        z.amplitude_offset *= -1
        z.amplitude *= -1
        return z
    
    def mirror_keep_amplitude_offset(self):
        z=self.clone()
        z.amplitude *= -1
        return z
        
    def mirror_freq(self):
        z=self.clone()
        z.phase_offset *= -1
        z.angular_frequency *= -1
        return z
        
    def __str__(self):
        return "y=%.2f+%.2f*sin(%.2f+%.2f*x)"%(self.amplitude_offset, self.amplitude, self.phase_offset, self.angular_frequency)
        
class WholeBodyWalkerFunction:
    """
    Multi-joint walk function for RobotisMini  
    Creates SinusoidFunction for each joint with different parameters
    """
    def __init__(self, walking_params):
        self.parameters={}
        
        print walking_params
        for pn, pp in walking_params.iteritems():
            self.parameters[pn + '_amplitude']=pp[0]
            self.parameters[pn + '_amplitude_offset']=pp[1]
            self.parameters[pn + '_phase_offset']=pp[2]

        self.parameters["step_frequency"]=math.pi
        
        self.parameters["vx_amplitude"]=0.5
        self.parameters["vy_amplitude"]=0.5
        self.parameters["vt_amplitude"]=0.4
        
        self.generate()
        
    def generate(self):
        """
        Build CPG functions for walk-on-spot (no translation or rotation, only legs up/down)
        """        
        
        self.pfn={} # phase joint functions    
        self.afn={} # anti phase joint functions

        ## Foot and hip -> Lateral motion
        foot_func=SinusoidFunction()
        foot_func.angular_frequency= self.parameters["step_frequency"]
        foot_func.amplitude= self.parameters["foot_amplitude"]
        foot_func.amplitude_offset= self.parameters["foot_amplitude_offset"]
        foot_func.phase_offset= self.parameters["foot_phase_offset"]
        self.pfn["l_foot_joint"]=foot_func        
        foot_func_af=foot_func.mirror()
        self.afn["l_foot_joint"]=foot_func_af
        
        hip_func=SinusoidFunction()
        hip_func.angular_frequency= self.parameters["step_frequency"]
        hip_func.amplitude= self.parameters["hip_amplitude"]
        hip_func.amplitude_offset= self.parameters["hip_amplitude_offset"]
        hip_func.phase_offset= self.parameters["hip_phase_offset"]
        self.pfn["l_hip_joint"]=hip_func
        hip_func_af=hip_func.mirror()
        self.afn["l_hip_joint"]=hip_func_af
        
        ## Thigh, ankle and knee -> Frontal motion
        thigh_func=SinusoidFunction()
        thigh_func.angular_frequency= self.parameters["step_frequency"]
        thigh_func.amplitude= self.parameters["thigh_amplitude"]
        thigh_func.amplitude_offset= self.parameters["thigh_amplitude_offset"]
        thigh_func.phase_offset= self.parameters["thigh_phase_offset"]
        self.pfn["l_thigh_joint"]=thigh_func
        thigh_func_af=thigh_func.mirror_keep_amplitude_offset()
        self.afn["l_thigh_joint"]=thigh_func_af
        
        ankle_func=SinusoidFunction()
        ankle_func.angular_frequency= self.parameters["step_frequency"]
        ankle_func.amplitude= self.parameters["ankle_amplitude"]
        ankle_func.amplitude_offset= self.parameters["ankle_amplitude_offset"]
        ankle_func.phase_offset= self.parameters["ankle_phase_offset"]
        self.pfn["l_ankle_joint"]=ankle_func
        ankle_func_af=ankle_func.mirror_keep_amplitude_offset()
        self.afn["l_ankle_joint"]=ankle_func_af
        
        knee_func=SinusoidFunction()
        knee_func.angular_frequency= self.parameters["step_frequency"]
        knee_func.amplitude= self.parameters["knee_amplitude"]
        knee_func.amplitude_offset= self.parameters["knee_amplitude_offset"]
        knee_func.phase_offset= self.parameters["knee_phase_offset"]
        self.pfn["l_knee_joint"]=knee_func
        knee_func_af=knee_func.mirror_keep_amplitude_offset()
        self.afn["l_knee_joint"]=knee_func_af
        
        #f3=SinusoidFunction()
        #f3.angular_frequency=self.parameters["step_frequency"]
        #f3.amplitude=self.parameters["step_amplitude"]
        #f3.amplitude_offset=self.parameters["step_amplitude_offset"]
        #self.pfn["l_thigh_joint"]= f3
        #f33=f3.clone()
        #f33.amplitude_offset = self.parameters["ankle_amplitude_offset"]
        #f33.amplitude = self.parameters["ankle_amplitude"]
        #self.pfn["l_ankle_joint"]=f33
        #f4=f3.mirror()
        ##f4.amplitude_offset -= 0.4
        #self.pfn["l_knee_joint"]=f4
        
        #f5=f3.mirror_keep_amplitude_offset()
        #self.afn["l_thigh_joint"]=f5
        
        #f6=f33.mirror_keep_amplitude_offset()
        #self.afn["l_ankle_joint"]=f6
        #f7=f5.mirror()
        ##f7.amplitude_offset -= 0.4
        #self.afn["l_knee_joint"]=f7
        
        self.generate_right()
        
        self.show()
              
    def generate_right(self):
        """
        Mirror CPG functions from left to right and antiphase right
        """
        l=[ v[2:] for v in self.pfn.keys()]
        for j in l:
            self.pfn["r_"+j]=self.afn["l_"+j].mirror_keep_amplitude_offset()
            self.pfn["r_"+j].phase_offset += math.pi
            self.afn["r_"+j]=self.pfn["l_"+j].mirror_keep_amplitude_offset()
            self.afn["r_"+j].phase_offset += math.pi
        
    def get(self, phase, x, velocity):
        """ Obtain the joint angles for a given phase, position in cycle (x 0,1)) and velocity parameters """
        angles={}
        for j in self.pfn.keys():
            if phase:
                v=self.pfn[j].get(x)
                angles[j]=v
            else:
                angles[j]=self.afn[j].get(x)
        self.apply_velocity(angles,velocity,phase,x)
        return angles
                   
    def show(self):
        """
        Display the CPG functions used
        """
        for j in self.pfn.keys():
            print j,"p",self.pfn[j],"a",self.afn[j]        
        print self.pfn["l_knee_joint"].amplitude_offset

    def apply_velocity(self, angles, velocity, phase, x):
        """ Modify on the walk-on-spot joint angles to apply the velocity vector"""
        
        # VX
        v=velocity[0]*self.parameters["vx_amplitude"]
        d=(x*2-1)*v
        if phase:
            angles["l_thigh_joint"]+=d
            angles["l_ankle_joint"]+=d
            angles["r_thigh_joint"]+=d
            angles["r_ankle_joint"]+=d
        else:
            angles["l_thigh_joint"]-=d
            angles["l_ankle_joint"]-=d
            angles["r_thigh_joint"]-=d
            angles["r_ankle_joint"]-=d

        # VY
        v=velocity[1]*self.parameters["vy_amplitude"]
        d=(x)*v
        d2=(1-x)*v
        if v>=0:
            if phase:
                angles["l_hip_joint"]-=d
                angles["l_foot_joint"]-=d
                angles["r_hip_joint"]+=d
                angles["r_foot_joint"]+=d
            else:
                angles["l_hip_joint"]-=d2
                angles["l_foot_joint"]-=d2
                angles["r_hip_joint"]+=d2
                angles["r_foot_joint"]+=d2
        else:
            if phase:
                angles["l_hip_joint"]+=d2
                angles["l_foot_joint"]+=d2
                angles["r_hip_joint"]-=d2
                angles["r_foot_joint"]-=d2
            else:
                angles["l_hip_joint"]+=d
                angles["l_foot_joint"]+=d
                angles["r_hip_joint"]-=d
                angles["r_foot_joint"]-=d
                
        ## VT
        #v=velocity[2]*self.parameters["vt_amplitude"]
        #d=(x)*v
        #d2=(1-x)*v
        #if v>=0:
            #if phase:
                #angles["j_pelvis_l"]=-d
                #angles["j_pelvis_r"]=d
            #else:
                #angles["j_pelvis_l"]=-d2
                #angles["j_pelvis_r"]=d2
        #else:
            #if phase:
                #angles["j_pelvis_l"]=d2
                #angles["j_pelvis_r"]=-d2
            #else:
                #angles["j_pelvis_l"]=d
                #angles["j_pelvis_r"]=-d

class Walker:
    """
    Class for making RobotisMini walk
    """
    def __init__(self, robotis_mini_ci, real_robot):
        
        self.robotis_mini_ci=robotis_mini_ci
        
        self.real_robot = real_robot
        
        self.displacing=False #When the robot is walking AND displacing in the plane (velocity is non-zero)
        self.walking=False  #When the robot is walking: moving up and down the legs
        self.velocity=[0,0,0]
        
        #Default walking params
        self.walking_params = {}
        self.walking_params['foot'] = [0.4,0,0]
        self.walking_params['ankle'] = [-0.01,-0.20,0]
        self.walking_params['knee'] = [0.4,0.7,0]
        self.walking_params['thigh'] = [-0.4,-0.7,0]
        self.walking_params['hip'] = [0.4,0,0]
        
        self.wb_walkerfunc=WholeBodyWalkerFunction(self.walking_params)

        self.initial_wq = self.wb_walkerfunc.get(True, 0, [0,0,0])  #First joint configuration to start the walking motion

        print "__init__:initial_wq"
        j_names=self.initial_wq.keys()
        for jn in j_names:
            print jn + str(":") + str(self.initial_wq[jn])
        
        self._th_walk=None  #Walking thread
        
        self._cycle_period = 10 #seconds

        self._sub_cmd_vel=rospy.Subscriber(robotis_mini_ci.ns+"cmd_vel", Twist,self._cb_cmd_vel, queue_size=1)
        self._sub_cmd_stop=rospy.Subscriber(robotis_mini_ci.ns+"stop_srv", Empty,self._cb_cmd_stop, queue_size=1)
        self._sub_cmd_restart=rospy.Subscriber(robotis_mini_ci.ns+"restart_srv", Empty,self._cb_cmd_restart, queue_size=1)
        self._sub_cmd_restart=rospy.Subscriber(robotis_mini_ci.ns+"walking_params", Float64MultiArray,self._cb_new_walking_params, queue_size=1)
        
        if not self.real_robot:
            self.pubs = {}
            # Wait until the joints have been populated
            while self.robotis_mini_ci.q_names is None:
                time.sleep(1)
            for jn in self.robotis_mini_ci.q_names:
                self.pubs[jn] = rospy.Publisher('/robotis_mini/' + jn + '_position_controller/command', Float64, queue_size=1)
            rospy.loginfo("Waiting for gazebo services")
            rospy.wait_for_service('/gazebo/pause_physics')
            self.pause_simulation_srv = rospy.ServiceProxy('/gazebo/pause_physics', std_srvs.srv.Empty)
            rospy.wait_for_service('/gazebo/reset_world')
            self.reset_world_srv = rospy.ServiceProxy('/gazebo/reset_world', std_srvs.srv.Empty)
            rospy.wait_for_service('/gazebo/unpause_physics')
            self.unpause_simulation_srv = rospy.ServiceProxy('/gazebo/unpause_physics', std_srvs.srv.Empty)

    def _cb_new_walking_params(self,msg):
        """
        Processes a new set of parameters
        """
        print "Walker new set of parameters received"
        self._cycle_period = msg.data[0]
        self.walking_params['foot'] = [msg.data[1],msg.data[2],msg.data[3]]
        self.walking_params['ankle'] = [msg.data[4],msg.data[5],msg.data[6]]
        self.walking_params['knee'] = [msg.data[7],msg.data[8],msg.data[9]]
        self.walking_params['thigh'] = [msg.data[10],msg.data[11],msg.data[12]]
        self.walking_params['hip'] = [msg.data[13],msg.data[14],msg.data[15]]
        
        self.wb_walkerfunc=WholeBodyWalkerFunction(self.walking_params)

        self.initial_wq = self.wb_walkerfunc.get(True, 0, [0,0,0])  #First joint configuration to start the walking motion

        print "initial_wq"
        j_names=self.initial_wq.keys()
        for jn in j_names:
            print jn + str(":") + str(self.initial_wq[jn])
            
    def _cb_cmd_restart(self,msg):
        """
        Processes cmd_restart and to start a new trial
        """
        print "Walker restart command received"
        
        #Stop the running thread
        while self.displacing or self.walking or self._th_walk:
            rospy.loginfo('Stopping walking thread')
            self.stop()
            
        #If the robot is simuated -> send to initial configuration
        if not self.real_robot:
            rospy.loginfo("Sending robot to zero configuration")
            for jn in self.robotis_mini_ci.q_names:
                self.pubs[jn].publish(0.0)
        
            time.sleep(1)
        
            #If the robot is simulated -> reset simulation
            try:
               self.pause_simulation_srv()
               rospy.loginfo( "Paused gazebo")
               time.sleep(1)
               self.reset_world_srv()
               rospy.loginfo( "Reseting gazebo")
               time.sleep(1)
               self.unpause_simulation_srv()
               rospy.loginfo( "Unpaused gazebo")
               time.sleep(1)
            except rospy.ServiceException, e:
               print "Service call failed: %s"%e
        
    def _cb_cmd_vel(self,msg):
        """
        Processes cmd_vel and update walker speed
        """
        print "Walker velocity command received: ",msg
        vx=msg.linear.x
        vy=msg.linear.y
        vt=msg.angular.z
        self.start()
        self.set_desired_velocity(vx,vy,vt)
    
    def _cb_cmd_stop(self,msg):
        """
        Processes cmd_stop
        """
        print "Walker stop command received: "
        self.stop()
        
    def goto_initial_wq(self):
        """
        If not there yet, go to initial walking configuration
        """
        rospy.loginfo("Going to initial walking configuration")
        while self.get_qdist_to_initial_wq()>0.1:
            rospy.loginfo("Commanding to go to initial walking configuration")
            print "Initial configuration"
            print self.initial_wq
            self.robotis_mini_ci.set_qd_interpolated(self.initial_wq, 2)
            rospy.sleep(2)   
        rospy.loginfo("Initial walking configuration reached")
        print "Distance",self.get_qdist_to_initial_wq() 
        
    def start(self):
        if not self.displacing:
            self.displacing=True                    
            self.goto_initial_wq()
            self._th_walk=Thread(target=self._do_walk)
            self._th_walk.start()
            self.walking=True
    
    def stop(self):
        if self.displacing:
            self.walking=False
            rospy.loginfo("Waiting for stopped")
            while not rospy.is_shutdown() and self._th_walk is not None:
                rospy.sleep(0.1)                
            rospy.loginfo("Stopped")
            self.displacing=False
            
    def set_desired_velocity(self,x,y,t):
        self.desired_velocity=[x,y,t]

    def _do_walk(self):
        """
        Main walking loop, smoothly update velocity vectors and apply corresponding joint configurations
        """
        rospy.loginfo("Started walking thread")
        wb_walkerfunc=self.wb_walkerfunc
        
        # Global walk loop
        n=50
        print "Thread rate", 1.0/(self._cycle_period/(2.0*n))
        r=rospy.Rate(1.0/(self._cycle_period/(2.0*n)))
        p=True
        i=0
        self.current_velocity=[0,0,0]
        while not rospy.is_shutdown() and (self.walking or i<n or self.is_displacing()):
            if not self.walking:
                self.desired_velocity=[0,0,0]
            #if not self.is_displacing() and i==0: # Do not move if nothing to do and already at 0
            #    self.update_current_velocity(self.desired_velocity, n)
            #    r.sleep()
            #    continue
            x=float(i)/n            
            qd_curr=wb_walkerfunc.get(p, x, self.current_velocity)
            self.update_current_velocity(self.desired_velocity, n)
            self.robotis_mini_ci.set_qd(qd_curr)
            i+=1
            if i>n:
                i=0
                p=not p
            r.sleep()
        rospy.loginfo("Finished walking thread")
        
        self._th_walk=None

    def is_displacing(self):
        """
        Checks if the current velocity is not zero and returns True in that case
        """
        e=0.02
        for v in self.current_velocity:
            if abs(v)>e: return True
        return False
                    
    def update_current_velocity(self, target_velocity, n):
        """
        A pseudo-interpolation to a target velocity
        """
        a=3/float(n)
        b=1-a
        self.current_velocity=[a*tv+b*cv for (tv,cv) in zip(target_velocity, self.current_velocity)]
        
    def get_qdist_to_initial_wq(self):
        """
        Computes the absolute distance between the current robot joint state and the initial walking configuration
        """
        current_q=self.robotis_mini_ci.get_q()
        return get_distance(self.initial_wq, current_q)

def get_distance(qa_dict, qb_dict):
    """
    Computes sum of absolute distances between two sets of joints represented as dictionaries of (jointName, jointConfiguration)
    """
    d=0
    j_names=qa_dict.keys()
    if len(j_names)==0:
        rospy.loginfo("Length is 0")
        return 0
    for jn in j_names:
        d+=abs(qb_dict[jn]-qa_dict[jn])
    d/=len(j_names)
    return d

if __name__=="__main__":
    rospy.init_node("walker")
    
    parser = argparse.ArgumentParser(description='Walker trajectory generator')
    parser.add_argument('--real',action='store_true', help='define when using the real robot')
    
    options, args = parser.parse_known_args()
    
    if options.real:
        rospy.loginfo("Real Robot!")
    else:
        rospy.loginfo("Simulated Robot!")
    
    rospy.loginfo("Instantiating RobotisMini RobotisMiniControlInterface")
    robotis_mini_ci=RobotisMiniControlInterface(real_robot=options.real)
    rospy.loginfo("Instantiating RobotisMini Walker")
    walker=Walker(robotis_mini_ci, options.real)
 
    rospy.loginfo("RobotisMini Walker Ready")
    while not rospy.is_shutdown():
        time.sleep(1)
