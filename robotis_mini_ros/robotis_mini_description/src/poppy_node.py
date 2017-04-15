#!/usr/bin/env python


#~ THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#~ WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#~ MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#~ ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#~ WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
#~ OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
#~ CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np


#pypot robot object, created from the /node_name/creature parameter, with RobotisMiniPoppy as default
global poppy
poppy = None 


def JointStateWrite(data):
    for i in range(len(data.name)):
        found = False
        for m in poppy.motors:
            if m.name == data.name[i]:
                found = True
                if len(data.position) > 0:
                    m.goal_position = np.rad2deg(data.position[i])
                    if  abs(data.position[i]) > 0. and m.compliant:
                        m.compliant = False
                if len(data.velocity) > 0:
                    m.goal_speed = np.rad2deg(data.velocity[i])
                    if  abs(data.velocity[i]) > 0. and m.compliant:
                        m.compliant = False
                if len(data.effort) > 0:
                    m.max_torque = data.effort[i]
                    if  abs(data.effort[i]) > 0. and m.compliant:
                        m.compliant = False
                break
        if not found:
            print "ERROR, unknown motor ",data.name[i]

        
def usePrimitive(data, args):

    p = getattr(poppy, args[0])
    if args[1] == "start":
        p.start()
    else:
        p.stop()


def poppy_node():
    #start node
    rospy.init_node('poppy_node')
    rate = rospy.Rate(10) 

    #from creature param, create creature object
    creature = rospy.get_param(rospy.get_name()+'/creature','robotis-mini-poppy')
    global poppy
        
    if creature.endswith(".json"):
        import pypot.robot
        poppy = pypot.robot.from_config(creature)
    else:
        libraryName = creature.replace('-', '_')

        objectName = creature.replace('-', " ")
        objectName = objectName.title()
        objectName = objectName.replace(" ", "")
    
        from importlib import import_module

        mod = import_module(libraryName )
        met = getattr(mod, objectName)
    
        poppy = met()

    #set motors info as params
    print "Robotis Mini joints:"
    for m in poppy.motors:
        print m.name
        rospy.set_param(rospy.get_name()+'/motor/'+m.name+'/id', m.id)
        rospy.set_param(rospy.get_name()+'/motor/'+m.name+'/model', m.model)
        rospy.set_param(rospy.get_name()+'/motor/'+m.name+'/direct', m.direct)
        rospy.set_param(rospy.get_name()+'/motor/'+m.name+'/offset', np.deg2rad(m.offset).item())
        rospy.set_param(rospy.get_name()+'/motor/'+m.name+'/upper_limit', np.deg2rad(m.upper_limit).item())
        rospy.set_param(rospy.get_name()+'/motor/'+m.name+'/lower_limit', np.deg2rad(m.lower_limit).item())

    #create publishers for motors present_position, present_speed, present_load and goal_position and goal_speed and max_torque
    pubs = {}
        
    pubs[rospy.get_name()+'/motors/read_present'] = rospy.Publisher(rospy.get_name()+'/motors/read_present', JointState, queue_size=10)
    pubs['/robotis_mini/joint_states'] = rospy.Publisher('/robotis_mini/joint_states', JointState, queue_size=10)
    pubs[rospy.get_name()+'/motors/read_goal'] = rospy.Publisher(rospy.get_name()+'/motors/read_goal', JointState, queue_size=10)
    
    #subscribe to topic to change the goal_position, goal_speed and compliance
    rospy.Subscriber(rospy.get_name()+'/motors/write', JointState, JointStateWrite)   
    rospy.Subscriber('/robotis_mini/goal', JointState, JointStateWrite)   
    
    print "Robotis Mini primitives:"
    for p in poppy.primitives:
        print p.name
        rospy.Subscriber(rospy.get_name()+'/primitive/'+p.name+'/start', String, usePrimitive, callback_args=[p.name, "start"])   
        rospy.Subscriber(rospy.get_name()+'/primitive/'+p.name+'/stop', String, usePrimitive, callback_args=[p.name, "stop"])  

    while not rospy.is_shutdown():
        
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for m in poppy.motors:           
            msg.name.append(m.name)
            msg.position.append(np.deg2rad(m.present_position))
            msg.velocity.append(np.deg2rad(m.present_speed))
            msg.effort.append(m.present_load)

        pubs[rospy.get_name()+'/motors/read_present'].publish(msg)
        pubs['/robotis_mini/joint_states'].publish(msg) #In standard topic
        
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for m in poppy.motors:           
            msg.name.append(m.name)
            msg.position.append(np.deg2rad(m.goal_position))
            msg.velocity.append(np.deg2rad(m.goal_speed))
            msg.effort.append(m.torque_limit)

        pubs[rospy.get_name()+'/motors/read_goal'].publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        poppy_node()
    except rospy.ROSInterruptException:
        pass
