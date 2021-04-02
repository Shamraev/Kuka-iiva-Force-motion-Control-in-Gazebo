#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from math import *

import numpy as np

import PyKDL as kdl
import kdl_parser_py.urdf

dt = 0.01

# Parameters for PD-controller with gravity compensation
KP = [50.0, 150.0, 40.0, 20.0, 18.0, 18.0, 15.0]
KD = [5.0, 25.0, 5.0, 2.0, 1.0, 1.0, 0.5]

torque_controller_pub = []
q = kdl.JntArray(7)
dq = kdl.JntArray(7)
ddq = kdl.JntArray(7)
meas_torques = kdl.JntArray(7)
grav_torques = kdl.JntArray(7)

def iiwaStateCallback(data):
    global q, dq, ddq
    for i in range(0,7):
        q[i] = data.position[i]
        dq[i] = data.velocity[i]
        ddq[i] = 0.0
        meas_torques[i] = data.effort[i]

def startControllers():
    # Start torque controllers. If you want to use position controllers, change 'joint1_torque_controller' to 'joint1_position_controller' and and so on.
    rospy.wait_for_service('/iiwa/controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy('/iiwa/controller_manager/switch_controller', SwitchController)
        ret = switch_controller(start_controllers=['joint1_torque_controller', 'joint2_torque_controller', 'joint3_torque_controller',
                                 'joint4_torque_controller', 'joint5_torque_controller', 'joint6_torque_controller', 'joint7_torque_controller'],
                                stop_controllers=[],
                                strictness=SwitchControllerRequest.STRICT)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def main():
    rospy.init_node('main_test', anonymous=True)
    rospy.Subscriber("/iiwa/joint_states", JointState, iiwaStateCallback)

    # Initialize publishers for send computation torques.
    for i in range(0,7):
        torque_controller_pub.append(rospy.Publisher("/iiwa/joint" + str(i+1) + "_torque_controller/command", Float64, queue_size=10))

    rate = rospy.Rate(int(1.0/dt))

    # Get parameters for kinematic from setup.yaml file
    base_link = rospy.get_param("/profi2021_master_solution/base_link_name")
    tool_link = rospy.get_param("/profi2021_master_solution/tool_link_name")
    force_desired = rospy.get_param("/profi2021_master_solution/force_desired")
    urdf = rospy.get_param("iiwa_urdf_model")

    rospy.loginfo("tip_name:  %s" % tool_link)
    rospy.loginfo("root_name: %s" % base_link)

    rospy.loginfo("Roslaunch param desired force:  %f" % force_desired)

    # Generate kinematic model for orocos_kdl
    (ok, tree) = kdl_parser_py.urdf.treeFromString(urdf)
    chain = tree.getChain(base_link, tool_link)
    L = np.transpose(np.array([[1, 1, 1, 0.01, 0.01, 0.01]]))
    iksolverpos = kdl.ChainIkSolverPos_LMA(chain, L)

    # Generate dynamic model for orocos_kdl
    grav = kdl.Vector(0, 0, -9.82)
    dyn_model = kdl.ChainDynParam(chain, grav)

    startControllers()

    torq_msg = Float64()
    counter = 0
    u = kdl.JntArray(7)
    q_dest = kdl.JntArray(7)
    frame_dest = kdl.Frame()

    # Setting up initial point
    frame_dest.p[0] = 0.4
    frame_dest.p[1] = 0.0
    frame_dest.p[2] = 0.1
    frame_dest.M = kdl.Rotation.RotY(3.14)

    ret = iksolverpos.CartToJnt(q, frame_dest, q_dest)

    while not rospy.is_shutdown():
        if counter*dt>3.0:
            frame_dest.p[0] = 0.1*sin(counter*dt) + 0.5
            frame_dest.p[1] = 0.1*cos(counter*dt);    # Start controllers
        ret = iksolverpos.CartToJnt(q, frame_dest, q_dest)
        rospy.loginfo("IK solution: %f, %f, %f, %f, %f, %f, %f", q_dest[0],  q_dest[1],  q_dest[2],  q_dest[3],  q_dest[4],  q_dest[5],  q_dest[6])
        dyn_model.JntToGravity(q, grav_torques)
        rospy.loginfo("Estimation torque on joins: %f, %f, %f, %f, %f, %f, %f\n" % (grav_torques[0], grav_torques[1],grav_torques[2], grav_torques[3], grav_torques[4], grav_torques[5], grav_torques[6]))
        for i in range(0,7):
            u[i] = KP[i]*(q_dest[i]-q[i]) - KD[i]*dq[i] + grav_torques[i]
            torq_msg.data = u[i]
            torque_controller_pub[i].publish(torq_msg)
        counter+=1
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
