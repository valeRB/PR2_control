import roslib
#roslib.load_manifest('cs225b')
import rosbag
import rospy
import numpy as np
from matplotlib.pyplot import *
import geometry_msgs.msg
import math


def read_data(bag):

    time = np.array([])
    r_x = np.array([])
    r_y = np.array([])
    r_z = np.array([])
    
    for topic, msg, t in bag.read_messages(topics=['/contact_point_estimation/contact_point_estimate']):
        time = np.append(time, msg.header.stamp.to_sec())
        r_x = np.append(r_x, msg.point.x)
        r_y = np.append(r_y, msg.point.y)
        r_z = np.append(r_z, msg.point.z)
        
    return time, r_x, r_y, r_z



if __name__=='__main__':
    try:

        name_bag=raw_input("Copy the name of the bag here \n")
        bag = rosbag.Bag(name_bag+'.bag')
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

        figure('Contact Point Estimate all-in-one')
        title('Contact Pt Estimate')
        plot(tiempo, X, 'r')
        plot(tiempo, Y, 'g')
        plot(tiempo, Z, 'b')
        legend(('X','Y','Z'),'upper left')
        
        show()

    except rospy.ROSInterruptException:
        pass

