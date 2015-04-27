import roslib
#roslib.load_manifest('cs225b')
import rospy
import math
#import tf
#from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import WrenchStamped
import numpy as np

alpha = 1.222
pub_l = rospy.Publisher("/ft_transformed/lef_arm",WrenchStamped)
pub_r = rospy.Publisher("/ft_transformed/rig_arm",WrenchStamped)

# Initialization of variables for RIGHT ARM
wrench_aligned_r = WrenchStamped()

Fx_r = 0
Fy_r = 0
Fz_r = 0
Tx_r = 0
Ty_r = 0
Tz_r = 0
bias_x_r = 0
bias_y_r = 0
bias_z_r = 0
bias_computed_r = 0
counter_r = 0

# Initialization of variables for LEFT ARM
wrench_aligned_l = WrenchStamped()

Fx_l = 0
Fy_l = 0
Fz_l = 0
Tx_l = 0
Ty_l = 0
Tz_l = 0
bias_x_l = 0
bias_y_l = 0
bias_z_l = 0
bias_computed_l = 0
counter_l = 0

def callback_right_arm(msg):
    global wrench_aligned_r, pub_r, bias_computed_r, alpha
    global bias_x_r, bias_y_r, bias_z_r
    #force_x = np.append(force_x, msg.wrench.force.x)
    wrench_aligned_r = msg
    if bias_computed_r == 0:
        (bias_x_r, bias_y_r, bias_z_r, bias_computed_r) = get_bias_r(wrench_aligned_r)
        if counter_r == 1000:
            print 'R_ Bias X: ', bias_x_r
            print 'R_ Bias Y: ', bias_y_r
            print 'R_ Bias Z: ', bias_z_r
    else:
        unbiased_Fx = wrench_aligned_r.wrench.force.x - bias_x_r
        unbiased_Fy = wrench_aligned_r.wrench.force.y - bias_y_r
        wrench_aligned_r.wrench.force.z -= bias_z_r
        wrench_aligned_r.wrench.force.x = unbiased_Fx*math.cos(alpha) - unbiased_Fy*math.sin(alpha)
        wrench_aligned_r.wrench.force.y = -(unbiased_Fx*math.sin(alpha)) - (unbiased_Fy*math.cos(alpha))
        pub_r.publish(wrench_aligned_r)

def get_bias_r(wrench_aligned):
    global Fx_r, Fy_r, Fz_r, counter_r, bias_computed_r, bias_x_r, bias_y_r, bias_z_r
    Fx_r += wrench_aligned.wrench.force.x
    Fy_r += wrench_aligned.wrench.force.y
    Fz_r += wrench_aligned.wrench.force.z
    counter_r += 1
    if counter_r == 1000:
        bias_x_r = Fx_r / 1000.0
        bias_y_r = Fy_r / 1000.0
        bias_z_r = Fz_r / 1000.0
        bias_computed_r = 1
    return bias_x_r, bias_y_r, bias_z_r, bias_computed_r

def callback_left_arm(msg):
    global wrench_aligned_l, pub_l, bias_computed_l
    global bias_x_l, bias_y_l, bias_z_l
    #force_x = np.append(force_x, msg.wrench.force.x)
    wrench_aligned_l = msg

    if bias_computed_l == 0:
        (bias_x_l, bias_y_l, bias_z_l, bias_computed_l) = get_bias_l(wrench_aligned_l)
        if counter_l == 1000:
            print 'L_ Bias X: ', bias_x_l
            print 'L_ Bias Y: ', bias_y_l
            print 'L_ Bias Z: ', bias_z_l
    else:
        unbiased_Fx = wrench_aligned_l.wrench.force.x - bias_x_l
        unbiased_Fy = wrench_aligned_l.wrench.force.y - bias_y_l
        wrench_aligned_l.wrench.force.z -= bias_z_l
        wrench_aligned_l.wrench.force.x = unbiased_Fx*math.cos(alpha) - unbiased_Fy*math.sin(alpha)
        wrench_aligned_l.wrench.force.y = -(unbiased_Fx*math.sin(alpha)) - (unbiased_Fy*math.cos(alpha))
        pub_l.publish(wrench_aligned_l)

def get_bias_l(wrench_aligned):
    global Fx_l, Fy_l, Fz_l, counter_l, bias_computed_l, bias_x_l, bias_y_l, bias_z_l
    Fx_l += wrench_aligned.wrench.force.x
    Fy_l += wrench_aligned.wrench.force.y
    Fz_l += wrench_aligned.wrench.force.z
    counter_l += 1
    if counter_l == 1000:
        bias_x_l = Fx_l / 1000.0
        bias_y_l = Fy_l / 1000.0
        bias_z_l = Fz_l / 1000.0
        bias_computed_l = 1
    return bias_x_l, bias_y_l, bias_z_l, bias_computed_l

    
    
def transform_test():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('transform_test', anonymous=True)
    
    rospy.Subscriber("/ft/r_gripper_motor", WrenchStamped, callback_right_arm)
    rospy.Subscriber("/ft/l_gripper_motor", WrenchStamped, callback_left_arm)
    
    rospy.spin()
    
if __name__ == '__main__':
    
    transform_test()

    try:
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()
            
    except KeyboardInterrupt:
        pass
##    raw_input("Press a key to continue")
##    print 'size of force_x', len(force_x)
##    print 'element [1] of force_x', force_x[1]
##    print 'element [1] of force_x_tf', force_x_tf[1]
    



