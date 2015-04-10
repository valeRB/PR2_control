#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Valeria Reynaga

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

global group_l, group_r
global cont1, what

def move_group_python_interface():

  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  #print "============ Waiting for RVIZ..."
  #rospy.sleep(10)


  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  #print "============ Left Reference frame: %s" % group_l.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  #print "============ Left End effector: %s" % group_l.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  #print "============ Robot Groups:"
  #print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  #print "============ Printing robot state"
  #print robot.get_current_state()
  
  print "============"


  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plan 1"
  pose_target_l = geometry_msgs.msg.Pose()
  pose_target_l.orientation.w = 1.0
  pose_target_l.position.x = 0.625476
  pose_target_l.position.y = 0.188251
  pose_target_l.position.z = 0.960486
  group_l.set_pose_target(pose_target_l)
  
  pose_target_r = geometry_msgs.msg.Pose()
  pose_target_r.orientation.w = 1.0
  pose_target_r.position.x = 0.625476
  pose_target_r.position.y = -0.188251
  pose_target_r.position.z = 0.960486
  group_r.set_pose_target(pose_target_r)
  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  plan_r = group_r.plan()
  plan_l = group_l.plan()

  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(2)

  # Uncomment below line when working with a real robot
  # group.go(wait=True)
  
  ## Then, we will get the current set of joint values for the group
  group_variable_values_r = group_r.get_current_joint_values()
  group_variable_values_l = group_l.get_current_joint_values()
  print "=== left_arm joint values: ", group_variable_values_l
  print "=== right_arm joint values: ", group_variable_values_r

  ## Planning to a joint-space goal 
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## Let's set a joint space goal and move towards it. 
  ## First, we will clear the pose target we had just set.
  group_r.clear_pose_targets()
  group_l.clear_pose_targets()
  while True:
      arm = raw_input("Which arm do you want to control?\n r: right_arm  l: left_arm  q: quit \n ")
      if arm == "r":
          print "Right Arm control\n"
          control_arm(arm, group_variable_values_r)
          pass
      elif arm == "l":
          print "Left Arm control\n"
          control_arm(arm, group_variable_values_l)          
          pass
      elif arm == "q":
          print "Exit control of arms"
          break
      else:
          print "Invalid option, nothing done"
          pass
    
##  group.set_joint_value_target(group_variable_values)
##  for x in range(0, 3):
##      plan2 = group.plan()
##
##      print "============ Waiting while RVIZ displays plan2..."
##      rospy.sleep(2)


  ## END_TUTORIAL

  print "============ STOPPING"

def control_arm(arm, group_variable_values):
  while True:
      what=raw_input("Choose a joint to control for == %s == \n 0: Open/Close Shoulder \n 1: Up/Down upper arm \n 2: Roll whole arm \n 3: Fold elbow \n 4: Roll forearm \n 5: Up/Down wrist \n 6: Roll Gripper \n q: Quit \n" %arm)
      if what == "0":        
        while True:
            act = raw_input("0: Open/Close Shoulder \n m: more  z: less e: exit this control \n")
            cont1 = change_joint(int(what), act, group_variable_values, arm)
            if cont1 == 0:
                what = 0
                act = 0
                break
        pass
      elif what == "1":
        while True:
            act = raw_input("1: Upper arm \n m: more  z: less e: exit this control \n")
            cont1 = change_joint(int(what), act, group_variable_values, arm)
            if cont1 == 0:
                what = 0
                act = 0
                break
        pass
      elif what == "2":
        while True:
            act = raw_input("2: Roll whole arm \n m: more  z: less e: exit this control \n")
            cont1 = change_joint(int(what), act, group_variable_values, arm)
            if cont1 == 0:
                what = 0
                act = 0
                break
        pass
      elif what == "3":
        while True:
            act = raw_input("3: Fold elbow \n m: more  z: less e: exit this control \n")
            cont1 = change_joint(int(what), act, group_variable_values, arm)
            if cont1 == 0:
                what = 0
                act = 0
                break
        pass
      elif what == "4":
        while True:
            act = raw_input("4: Roll forearm \n m: more  z: less e: exit this control \n")
            cont1 = change_joint(int(what), act, group_variable_values, arm)
            if cont1 == 0:
                what = 0
                act = 0
                break
        pass
      elif what == "5":
        while True:
            act = raw_input("5: Up/Down Wrist \n m: more  z: less e: exit this control \n")
            cont1 = change_joint(int(what), act, group_variable_values, arm)
            if cont1 == 0:
                what = 0
                act = 0
                break        
        pass
      elif what == "6":
        while True:
            act = raw_input("6: Roll gripper \n m: more  z: less e: exit this control \n")
            cont1 = change_joint(int(what), act, group_variable_values, arm)
            if cont1 == 0:
                what = 0
                act = 0
                break
        pass
      elif what == "q":
        print "Exit control of %s arm" %arm
        break
      else:
        print "Invalid option, nothing done"
        pass
   

def change_joint(joint_number, action, group_variable_values, arm):
  global cont1
  if action == "m":
      group_variable_values[joint_number] = group_variable_values[joint_number] + 0.05
      print "== %s == Joint values: " %arm, group_variable_values
      cont1 = 1
      pass
  elif action == "z":
      group_variable_values[joint_number] = group_variable_values[joint_number] - 0.05
      print "== %s == Joint values: " %arm, group_variable_values
      cont1 = 1
      pass
  elif action == "e":
      print "Exit control of this joint"
      cont1 = 0
      print "== %s == Joint values: " %arm, group_variable_values
      pass
  else:
      print "Invalid option, nothing done"
      print "== %s == Joint values: " %arm, group_variable_values
      cont1 = 1
      pass
  if cont1 != 0:
      if arm == "r":
          group_r.set_joint_value_target(group_variable_values)
          plan2 = group_r.plan() # Active in simulation
      else:
          group_l.set_joint_value_target(group_variable_values)
          plan2 = group_l.plan() # Active in simulation          
      #group.go(wait=True) # Active when running real robot
  return cont1

if __name__=='__main__':
  try:
    #what = 0
    group_l = moveit_commander.MoveGroupCommander("left_arm")
    group_r = moveit_commander.MoveGroupCommander("right_arm")
    move_group_python_interface()

  except rospy.ROSInterruptException:
    pass

