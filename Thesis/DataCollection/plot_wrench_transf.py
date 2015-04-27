import rosbag
import rospy
import copy
import numpy as np
from matplotlib.pyplot import *
import geometry_msgs.msg
import tf_conversions.posemath as pm
import PyKDL
import math

alpha = -1.222

def read_data(bag, arm):
    '''
    Returns tuple (time, theta, theta_ref, angular_vel, angular_vel_ref,
    angular_acc, angular_acc_ref, s, gripper_pos_command, gripper_vel_command, gripper_pos)
    '''
    global alpha
    time = np.array([])
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0.0, 0.0, 1.22), PyKDL.Vector(0.0,0.0,0.0))
    force_x = np.array([])
    force_x_t = np.array([])
    force_y = np.array([])
    force_y_t = np.array([])
    force_z = np.array([])
    torque_x = np.array([])
    torque_x_t = np.array([])
    torque_y = np.array([])
    torque_y_t = np.array([])
    torque_z = np.array([])

    for topic, msg, t in bag.read_messages(topics=['/ft/'+arm+'_gripper_motor']):
        time = np.append(time, msg.header.stamp.to_sec())  
        force_x = np.append(force_x, msg.wrench.force.x)
        force_y = np.append(force_y, msg.wrench.force.y)
        #aligned_Fx = msg.wrench.force.x*math.cos(alpha) - msg.wrench.force.y*math.sin(alpha)
        #aligned_Fy = msg.wrench.force.x*math.sin(alpha) + msg.wrench.force.y*math.cos(alpha)
        #force_x_t = np.append(force_x_t, aligned_Fx)
        #force_y_t = np.append(force_y_t, aligned_Fy)
        
        #force_z = np.append(force_z, msg.wrench.force.z)
##        torque_x = np.append(torque_x, msg.wrench.torque.x)
##        torque_y = np.append(torque_y, msg.wrench.torque.y)
##        aligned_Tx = msg.wrench.torque.y*math.sin(alpha) + msg.wrench.torque.x*math.cos(alpha)
##        aligned_Ty = msg.wrench.torque.y*math.cos(alpha) - msg.wrench.torque.x*math.sin(alpha)
##        torque_x_t = np.append(torque_x_t, aligned_Tx)
##        torque_y_t = np.append(torque_y_t, aligned_Ty)
        
        #torque_z = np.append(torque_z, msg.wrench.torque.z)
        

        #return time, force_x, force_y, force_z, torque_x, torque_y, torque_z
    return time, force_x, force_y, #force_x_t, force_y_t, torque_x, torque_y, torque_x_t, torque_y_t


if __name__=='__main__':
    try:
##        l_time = np.linspace(0,100)
##        l_force_x = np.linspace(0,100)
##        l_force_y = np.linspace(0,100)
##        l_force_z = np.linspace(0,100)
##        l_torque_x = np.linspace(0,100)
##        l_torque_y = np.linspace(0,100)
##        l_torque_z = np.linspace(0,100)
##
##        r_time = np.linspace(0,100)
##        r_force_x = np.linspace(0,100)
##        r_force_y = np.linspace(0,100)
##        r_force_z = np.linspace(0,100)
##        r_torque_x = np.linspace(0,100)
##        r_torque_y = np.linspace(0,100)
##        r_torque_z = np.linspace(0,100)
         
        name_bag=raw_input("Copy the name of the bag here \n")
        bag = rosbag.Bag('Bags Sunday/'+name_bag+'.bag')
        #(r_time, r_force_x, r_force_y, r_force_x_t, r_force_y_t, r_torque_x, r_torque_y, r_torque_x_t, r_torque_y_t) = read_data(bag, 'r')
        #(l_time, l_force_x, l_force_y, l_force_x_t, l_force_y_t, l_torque_x, l_torque_y, l_torque_x_t, l_torque_y_t) = read_data(bag, 'l')
        (r_time, r_force_x, r_force_y) = read_data(bag, 'l')
        ##        (l_time, l_force_x, l_force_y, l_force_z, l_torque_x,
##                         l_torque_y, l_torque_z)= read_data(bag, 'l')
##        (r_time, r_force_x, r_force_y, r_force_z, r_torque_x,
##                         r_torque_y, r_torque_z)= read_data(bag, 'r')
        #print "First element [2] of l_force_y:  %f" %r_force_y[2]
        raw_input("Press any key to see the plots \n")
        # Plot data
        l_time = l_time - l_time[0]
        r_time = r_time - r_time[0]

        # Plot transformed force x
        figure('Transformation of force Right_arm')
        subplot(221)
        plot(r_time, r_force_x), title('R Force X'), ylabel('[N]')
        subplot(222)
        plot(r_time, r_force_x_t), title('R Force X Transformed'), ylabel('[N]')
        subplot(223)
        plot(r_time, r_force_y), title('R Force Y'), ylabel('[N]')
        subplot(224)
        plot(r_time, r_force_y_t), title('R Force Y Transformed'), ylabel('[N]')

        figure('Transformation of force Left_arm')
        subplot(221)
        plot(l_time, l_force_x), title('L Force X'), ylabel('[N]')
        subplot(222)
        plot(l_time, l_force_x_t), title('L Force X Transformed'), ylabel('[N]')
        subplot(223)
        plot(l_time, l_force_y), title('L Force Y'), ylabel('[N]')
        subplot(224)
        plot(l_time, l_force_y_t), title('L Force Y Transformed'), ylabel('[N]')

##        figure('Transformation of Torque Right_arm')
##        subplot(221)
##        plot(r_time, r_torque_x), title('R torque X'), ylabel('[Nm]')
##        subplot(222)
##        plot(r_time, r_torque_x_t), title('R torque X Transformed'), ylabel('[Nm]')
##        subplot(223)
##        plot(r_time, r_torque_y), title('R torque Y'), ylabel('[Nm]')
##        subplot(224)
##        plot(r_time, r_torque_y_t), title('R torque Y Transformed'), ylabel('[Nm]')
##
##        figure('Transformation of Torque Left_arm')
##        subplot(221)
##        plot(l_time, l_torque_x), title('L torque X'), ylabel('[Nm]')
##        subplot(222)
##        plot(l_time, l_torque_x_t), title('L torque X Transformed'), ylabel('[Nm]')
##        subplot(223)
##        plot(l_time, l_torque_y), title('L torque Y'), ylabel('[Nm]')
##        subplot(224)
##        plot(l_time, l_torque_y_t), title('L torque Y Transformed'), ylabel('[Nm]')

##        # Plot Right Arm info
##        figure('Right Arm')
##        subplot(231)
##        plot(r_time, r_force_x), title('R Force X'), ylabel('[N]')
##
##        subplot(232)
##        plot(r_time, r_force_y), title('R Force Y'), ylabel('[N]')
##
##        subplot(233)
##        plot(r_time, r_force_z), title('R Force Z'), ylabel('[N]')
##
##        subplot(234)
##        plot(r_time, r_torque_x), title('R Torque X'), ylabel('[Nm]')
##
##        subplot(235)
##        plot(r_time, r_torque_y), title('R Torque Y'), ylabel('[Nm]')
##
##        subplot(236)
##        plot(r_time, r_torque_z), title('R Torque Z'), ylabel('[Nm]')
##        
##        figure('Left Arm')
##        
##        subplot(231)
##        plot(l_time, l_force_x), title('L Force X'), ylabel('[N]')
##
##        subplot(232)
##        plot(l_time, l_force_y), title('L Force Y'), ylabel('[N]')
##
##        subplot(233)
##        plot(l_time, l_force_z), title('L Force Z'), ylabel('[N]')
##
##        subplot(234)
##        plot(l_time, l_torque_x), title('L Torque X'), ylabel('[Nm]')
##
##        subplot(235)
##        plot(l_time, l_torque_y), title('L Torque Y'), ylabel('[Nm]')
##
##        subplot(236)
##        plot(l_time, l_torque_z), title('L Torque Z'), ylabel('[Nm]')
        
        show()

    except rospy.ROSInterruptException:
        pass

