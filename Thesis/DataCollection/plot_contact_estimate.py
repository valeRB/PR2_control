import roslib
#roslib.load_manifest('cs225b')
import rosbag
import rospy
import numpy as np
from matplotlib.pyplot import *
import geometry_msgs.msg
import math

gnd_point_x = 0
gnd_point_y = 0
gnd_point_z = 0.63

def read_data(bag):

    time = np.array([])
    r_x = np.array([])
    r_y = np.array([])
    r_z = np.array([])
    
    for topic, msg, t in bag.read_messages(topics=['/contact_point_estimation/contact_point_estimate']):#,'/ft/r_gripper_motor']):

        if abs(msg.point.x) > 3 or abs(msg.point.y) > 3 or abs(msg.point.z) > 3:
            print 'Something went wrong with the estimator =('
            break
        else:
            
            time = np.append(time, msg.header.stamp.to_sec())
            r_x = np.append(r_x, msg.point.x)
            r_y = np.append(r_y, msg.point.y)
            r_z = np.append(r_z, msg.point.z)
            
        
        
    return time, r_x, r_y, r_z



if __name__=='__main__':
    try:
        figure('Contact Point Estimate all-in-one')
        title('Contact Pt Estimate')
        name_bag=raw_input("Copy the name of the bag here \n")
##        bag = rosbag.Bag(name_bag+'.bag')
        bag = rosbag.Bag('../../../BagsMay1/ct_pt_bag/'+name_bag+'.bag')
        #read_data(bag)
        (tiempo, X, Y, Z) = read_data(bag)
        raw_input("Press any key to see the plots \n")
        # Plot data
        tiempo = tiempo - tiempo[0]
        

        # Plot contact point estimate
        figure('Contact Point Estimation')
        subplot(311)
        plot(tiempo, X), title('X'), ylabel('[m]')

        subplot(312)
        plot(tiempo, Y), title('Y'), ylabel('[m]')

        subplot(313)
        plot(tiempo, Z), title('Z'), ylabel('[m]')

        ref_x = np.zeros(len(tiempo)) + gnd_point_x
        ref_y = np.zeros(len(tiempo)) + gnd_point_y
        ref_z = np.zeros(len(tiempo)) + gnd_point_z

        figure('Contact Point Estimate all-in-one')
        title('Contact Pt Estimate')
        plot(tiempo, X, 'r')
        plot(tiempo, Y, 'g')
        plot(tiempo, Z, 'b')
        plot(tiempo, ref_x , '--r')
        plot(tiempo, ref_y , '--g')
        plot(tiempo, ref_z , '--b')
        legend(('X','Y','Z'),'upper left')


##        figure('Error')
##        title('Estimation error')
##        plot(tiempo, X - gnd_point_x, 'r')
##        plot(tiempo, Y - gnd_point_y, 'g')
##        plot(tiempo, Z - gnd_point_z, 'b')
##        legend(('X Error','Y Error','Z Error'),'upper left')

        
        show()

    except rospy.ROSInterruptException:
        pass

