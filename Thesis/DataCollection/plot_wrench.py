import rosbag
import rospy
import copy
import numpy as np
from matplotlib.pyplot import *
import geometry_msgs.msg
import math

alpha = 1.222

def read_data(bag, arm):
    '''
    Returns tuple (time, theta, theta_ref, angular_vel, angular_vel_ref,
    angular_acc, angular_acc_ref, s, gripper_pos_command, gripper_vel_command, gripper_pos)
    '''    
    time = np.array([])
    force_x = np.array([])
    force_y = np.array([])
    force_z = np.array([])
    torque_x = np.array([])
    torque_y = np.array([])
    torque_z = np.array([])


    for topic, msg, t in bag.read_messages(topics=['/ft/'+arm+'_gripper_motor']):        
        time = np.append(time, msg.header.stamp.to_sec())
        force_x = np.append(force_x, msg.wrench.force.x)
        force_y = np.append(force_y, msg.wrench.force.y)
        force_z = np.append(force_z, msg.wrench.force.z)
        torque_x = np.append(torque_x, msg.wrench.torque.x)
        torque_y = np.append(torque_y, msg.wrench.torque.y)
        torque_z = np.append(torque_z, msg.wrench.torque.z)

    return time, force_x, force_y, force_z, torque_x, torque_y, torque_z

def remove_bias(Fx, Fy, Fz): # Tx, Ty, Tz):
    suma_Fx = 0
    suma_Fy = 0
    suma_Fz = 0
    counter = 0
    for i in range (0,999):
        suma_Fx += Fx[i]
        suma_Fy += Fy[i]
        suma_Fz += Fz[i]
        counter += 1
    counter += 1
    bias_Fx = suma_Fx / counter
    bias_Fy = suma_Fy / counter
    bias_Fz = suma_Fz / counter
    print 'bias_Fx = ', bias_Fx
    print 'bias_Fy = ', bias_Fy
    print 'bias_Fz = ', bias_Fz
    Fx -= bias_Fx
    Fy -= bias_Fy
    Fz -= bias_Fz
    return Fx, Fy, Fz

def transform_vectors(Fx, Fy):
    global alpha
    a_force_x = np.array([])
    a_force_y = np.array([])
    for i in range(0, len(Fx)):
        a_Fx = Fx[i]*math.cos(alpha) - (Fy[i]*math.sin(alpha))
        a_Fy = - (Fx[i]*math.sin(alpha)) - (Fy[i]*math.cos(alpha))
        a_force_x = np.append(a_force_x, a_Fx)
        a_force_y = np.append(a_force_y, a_Fy)
    return a_force_x, a_force_y

if __name__=='__main__':
    try:
         
        name_bag=raw_input("Copy the name of the bag here \n")
        bag = rosbag.Bag(name_bag+'.bag')
##        (l_time, l_force_x, l_force_y, l_force_z, l_torque_x,
##                         l_torque_y, l_torque_z)= read_data(bag, 'l')
        (r_time, r_force_x, r_force_y, r_force_z, r_torque_x,
                    r_torque_y, r_torque_z)= read_data(bag, 'r')
        # Get bias for both arms in sensor
        (r_Fx, r_Fy, r_Fz) = remove_bias(r_force_x, r_force_y, r_force_z)
##        (l_Fx, l_Fy, l_Fz) = remove_bias(l_force_x, l_force_y, l_force_z)
        (r_tf_Fx, r_tf_Fy) = transform_vectors(r_Fx, r_Fy)
        raw_input("Press any key to see the plots \n")
        # Plot data
##        l_time = l_time - l_time[0]
        r_time = r_time - r_time[0]

        
        # Plot Right Arm info
        figure('Right Arm - Original Data')
        subplot(231)
        plot(r_time, r_force_x), title('R Force X'), ylabel('[N]')

        subplot(232)
        plot(r_time, r_force_y), title('R Force Y'), ylabel('[N]')

        subplot(233)
        plot(r_time, r_force_z), title('R Force Z'), ylabel('[N]')

        subplot(234)
        plot(r_time, r_torque_x), title('R Torque X'), ylabel('[Nm]')

        subplot(235)
        plot(r_time, r_torque_y), title('R Torque Y'), ylabel('[Nm]')

        subplot(236)
        plot(r_time, r_torque_z), title('R Torque Z'), ylabel('[Nm]')
                
        # Plot Right Arm info
        figure('Right Arm - Bias Removed')
        subplot(231)
        plot(r_time,  r_Fx), title('R Force X'), ylabel('[N]')

        subplot(232)
        plot(r_time, r_Fy), title('R Force Y'), ylabel('[N]')

        subplot(233)
        plot(r_time, r_Fz), title('R Force Z'), ylabel('[N]')

        subplot(234)
        plot(r_time, r_torque_x), title('R Torque X'), ylabel('[Nm]')

        subplot(235)
        plot(r_time, r_torque_y), title('R Torque Y'), ylabel('[Nm]')

        subplot(236)
        plot(r_time, r_torque_z), title('R Torque Z'), ylabel('[Nm]')

                # Plot Right Arm info
        figure('Right Arm - Transformed Data')
        subplot(231)
        plot(r_time, r_tf_Fx), title('R Force X'), ylabel('[N]')

        subplot(232)
        plot(r_time, r_tf_Fy), title('R Force Y'), ylabel('[N]')

        subplot(233)
        plot(r_time, r_Fz), title('R Force Z'), ylabel('[N]')

        subplot(234)
        plot(r_time, r_torque_x), title('R Torque X'), ylabel('[Nm]')

        subplot(235)
        plot(r_time, r_torque_y), title('R Torque Y'), ylabel('[Nm]')

        subplot(236)
        plot(r_time, r_torque_z), title('R Torque Z'), ylabel('[Nm]')
        
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
##
##        figure('Left Arm - bias')
##        subplot(231)
##        plot(l_time, l_force_x - l_bias_Fx), title('L Force X'), ylabel('[N]')
##
##        subplot(232)
##        plot(l_time, l_force_y - l_bias_Fy), title('L Force Y'), ylabel('[N]')
##
##        subplot(233)
##        plot(l_time, l_force_z - l_bias_Fz), title('L Force Z'), ylabel('[N]')
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

