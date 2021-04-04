#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from math import *

import numpy as np

import PyKDL as kdl
import kdl_parser_py.urdf

import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

dt = 0.01

# Parameters for PD-controller with gravity compensation
KP = [50.0, 150.0, 40.0, 20.0, 18.0, 18.0, 15.0]
KD = [5.0, 25.0, 5.0, 2.0, 1.0, 1.0, 0.5]

torque_controller_pub = []
position_controller_pub = []
q = kdl.JntArray(7)
dq = kdl.JntArray(7)
ddq = kdl.JntArray(7)
meas_torques = kdl.JntArray(7)
grav_torques = kdl.JntArray(7)
Fz = 0
FzDef = -8.5
cv_bridge = CvBridge()
NextP = np.array([0, 0, 0])
angleZ = 0
Debug = False
QStart = q

def iiwaStateCallback(data):
    global q, dq, ddq
    for i in range(0,7):
        q[i] = data.position[i]
        dq[i] = data.velocity[i]
        ddq[i] = 0.0
        meas_torques[i] = data.effort[i]

def iiwaForceSensorCallback(data):
    global Fz
    Fz = -data.wrench.force.z #-8.5 N - default
    Fz = Fz - FzDef # Force to surface
def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(3)

def iiwaImageCallback(img_msg):
    global NextP, angleZ
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        if Debug:
            cv2.imshow("Original Image", cv_image)
        #Convert to grayscale and threshold
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 30])

        masking = cv2.inRange(hsv_img, lower_black, upper_black)

        image, contours, hierarchy = cv2.findContours(masking, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if len(contours)<=0: return None

        img = cv_image.copy()
        cv2.drawContours(img, contours, -1, (0, 0, 255), 5)
        if Debug:
            cv2.imshow("Black contours at the image", img)
            cv2.waitKey(1)

        # find the biggest countour (c) by the area
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)

        # draw the biggest contour (c) in green
        if Debug:
            output = cv_image.copy()
            cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.imshow("Black contour at the image_box", output)
            cv2.waitKey(1)

        pointS = []
        pointE = []
        # camera info
        h = 480
        w = 640
        if c.any(): # Found contour with max area
            yHLineS = h - 10 # horizontal Line
            yHLineE = h - 50 # horizontal Line
            curP = np.array([w/2,yHLineS])

            pointS.append(c[c[:,:,1] == yHLineS])
            pointE.append(c[c[:,:,1] == yHLineE])
            if len(pointS)<=0 or len(pointE)<=0 or pointS[0].size==0 or pointE[0].size==0:
                return None
            pS = np.array([pointS[0][0][0], pointS[0][0][1]])
            pE = np.array([pointE[0][0][0], pointE[0][0][1]])
            V = pE - pS
            cv_image = cv2.circle(cv_image, (pS[0],pS[1]), radius=10, color=(0, 0, 255), thickness=2)
            cv_image = cv2.circle(cv_image, (pE[0],pE[1]), radius=10, color=(0, 0, 255), thickness=5)
            cv_image = cv2.circle(cv_image, (curP[0],curP[1]), radius=10, color=(0, 255, 0), thickness=5)

            cv2.imshow("Points to go", cv_image)
            cv2.waitKey(1)

            # CS
            NextP[0],NextP[1] = -(pE[0] - w/2), -(h - pE[1]) # down, left from center of image - Center of CS
            # Angle
            V[1] = - V[1]
            angleZ = atan2(V[0], V[1])

    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e)) #loginfo

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

def switchToTorqueControllers():
    rospy.wait_for_service('/iiwa/controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy('/iiwa/controller_manager/switch_controller', SwitchController)
        ret = switch_controller(start_controllers=['joint1_torque_controller', 'joint2_torque_controller', 'joint3_torque_controller',
                                 'joint4_torque_controller', 'joint5_torque_controller', 'joint6_torque_controller', 'joint7_torque_controller'],
                                stop_controllers=['joint1_position_controller', 'joint2_position_controller', 'joint3_position_controller',
                                 'joint4_position_controller', 'joint5_position_controller', 'joint6_position_controller', 'joint7_position_controller'],

                                strictness=SwitchControllerRequest.STRICT)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def switchToPositionControllers():
    rospy.wait_for_service('/iiwa/controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy('/iiwa/controller_manager/switch_controller', SwitchController)
        ret = switch_controller(start_controllers=['joint1_position_controller', 'joint2_position_controller', 'joint3_position_controller',
                                 'joint4_position_controller', 'joint5_position_controller', 'joint6_position_controller', 'joint7_position_controller'],
                                stop_controllers=['joint1_torque_controller', 'joint2_torque_controller', 'joint3_torque_controller',
                                 'joint4_torque_controller', 'joint5_torque_controller', 'joint6_torque_controller', 'joint7_torque_controller'],
                                strictness=SwitchControllerRequest.STRICT)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def shutdown():
    # Return to start position
    print("Befor shutdown return the robot to start position!")
    switchToPositionControllers()
    pos_msg = Float64()
    for i in range(0,7):
        pos_msg.data = QStart[i]*0
        position_controller_pub[i].publish(pos_msg)
    time.sleep(6)
    switchToTorqueControllers()
def main():
    global angleZ
    rospy.on_shutdown(shutdown)
    rospy.init_node('main_test', anonymous=True)
    rospy.Subscriber("/iiwa/joint_states", JointState, iiwaStateCallback)
    rospy.Subscriber("/iiwa/camera1/image_raw", Image, iiwaImageCallback)
    rospy.Subscriber("/iiwa/state/CartesianWrench", WrenchStamped, iiwaForceSensorCallback)

    # Initialize publishers for send computation torques.
    for i in range(0,7):
        torque_controller_pub.append(rospy.Publisher("/iiwa/joint" + str(i+1) + "_torque_controller/command", Float64, queue_size=10))
        position_controller_pub.append(rospy.Publisher("/iiwa/joint" + str(i+1) + "_position_controller/command", Float64, queue_size=10))
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
    jntToJacSolver = kdl.ChainJntToJacSolver(chain)

    # Generate dynamic model for orocos_kdl
    grav = kdl.Vector(0, 0, -9.82)
    dyn_model = kdl.ChainDynParam(chain, grav)

    startControllers()

    torq_msg = Float64()
    counter = 0
    u = kdl.JntArray(7)
    q_dest = kdl.JntArray(7)
    frame_dest = kdl.Frame()

    QStart = q
    # Setting up initial point
    frame_dest.p[0] = 0.4
    frame_dest.p[1] = 0.0
    frame_dest.p[2] = 0.5
    frame_dest.M = kdl.Rotation.RotY(3.14)*kdl.Rotation.RotZ(-3.14/2)

    ret = iksolverpos.CartToJnt(q, frame_dest, q_dest)

    NextP[0] = frame_dest.p[0]
    NextP[1] = frame_dest.p[1]
    NextP[2] = frame_dest.p[2]
    needWait = 0
    startT = 0
    StartRotate = True
    while not rospy.is_shutdown():
        if counter*dt>1.5:
            frame_dest.p[2] = 0.25
        if counter*dt>3.0:
            frame_dest.p[2] = 0.1

        if counter*dt>5.0:
            # In Global CS
            p = kdl.Vector(NextP[1], NextP[0],0)*0.001*0.3*dt
            p = frame_dest*p
            if Debug:
                rospy.loginfo("nexP in local CS: (%f, %f)",NextP[0], NextP[1])
                rospy.loginfo("nexP in Global CS: (%f, %f)",p.x(), p.y())
                rospy.loginfo("CurP in Global CS: (%f, %f, %f)",frame_dest.p[0], frame_dest.p[1], frame_dest.p[2])
                rospy.loginfo("angleZ: %f", angleZ)
                rospy.loginfo("Fz: %f", Fz)

            [Rz, Ry, Rx] = frame_dest.M.GetEulerZYX()

            # Loop
            if StartRotate and  abs(Rz)<0.01:
                needWait = 2
                startT = counter*dt
                StartRotate = False

            if needWait==0 and counter*dt>7.0:
                frame_dest.p[0] = p.x()
                frame_dest.p[1] = p.y()
            elif counter*dt - startT>needWait: # wait
                needWait = 0
            else:
                angleZ = -3.14/2

            if counter*dt - startT>needWait+5:
                StartRotate = True

            frame_dest.M = frame_dest.M*kdl.Rotation.RotZ(angleZ*dt*2)

        ret = iksolverpos.CartToJnt(q, frame_dest, q_dest)
        dyn_model.JntToGravity(q, grav_torques)

        if force_desired>20: # Not checked for force_desired>20
            force_desired = 20

        Fzerr = (force_desired-Fz)

        J = kdl.Jacobian(7)
        jntToJacSolver.JntToJac(q, J)

        xdot = kdl.JntArray(6)
        for i in range(6):
            xdot[i] = 0;
            for j in range(7):
                xdot[i] += J[i,j] * dq[j]

        FFz = [0, 0, -Fzerr*0.1, 0, 0, 0]
        tau_Fz = kdl.JntArray(7)
        for i in range(0,7):

            for j in range(0,6):
                tau_Fz[i] += J[j,i] * FFz[j];

            u[i] = KP[i]*(q_dest[i]-q[i]) - KD[i]*dq[i] + grav_torques[i] + tau_Fz[i]
            torq_msg.data = u[i]
            torque_controller_pub[i].publish(torq_msg)

        counter+=1
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
