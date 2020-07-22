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
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL
import numpy as np

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

l = 1

def get_targets(x,y):
    r, phi = cart2pol(x,y)
    if r > l*np.cos(0.26) or r < l*np.cos(0.77):
        print '++++++ Not in Domain ++++++'
        return None
    theta_shoulder = np.arccos(r/l)
    theta_wrist = theta_shoulder + np.pi/2
    theta_elbow = np.pi/2 - 2*theta_shoulder
    return [phi, theta_shoulder, theta_elbow, theta_wrist]


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "braccio_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):

    print 'pos x?'
    tst = raw_input()
    if tst!='':
        x = float(tst)
    print 'pos y?'
    tst = raw_input()
    if tst!='':
        y = float(tst)

    joint_targets = get_targets(x,y)

    print joint_targets
    if joint_targets:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = joint_targets[0]
        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()

        joint_goal[1] = joint_targets[1]
        joint_goal[2] = joint_targets[2]
        joint_goal[3] = joint_targets[3]

        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()

        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    else:
        return False

  def go_to_home_state(self):

    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 1.15
    joint_goal[2] = 0.13
    joint_goal[3] = 2.29

    self.move_group.go(joint_goal, wait=True)

    self.move_group.stop()

    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 3.14

    self.move_group.go(joint_goal, wait=True)

    self.move_group.stop()

  def go_to_up_state(self):

    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 1.5708
    joint_goal[1] = 1.5708
    joint_goal[2] = 1.5708
    joint_goal[3] = 1.5708

    self.move_group.go(joint_goal, wait=True)

    self.move_group.stop()


  def print_pose(self):
    print self.move_group.get_current_pose().pose

def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()

    while True:
        print "============ instructions: p=print, c=control, h=home, u=up, q=quit"
        inp = raw_input()
        if inp=='q':
            break
        if inp=='p':
            tutorial.print_pose()
        if inp=='c':
            tutorial.go_to_joint_state()
        if inp=='h':
            tutorial.go_to_home_state()
        if inp=='u':
            tutorial.go_to_up_state()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
