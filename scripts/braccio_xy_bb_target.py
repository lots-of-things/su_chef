#!/usr/bin/env python

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Int16MultiArray
from sensor_msgs.msg import CompressedImage

from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL
import numpy as np
import cv2
import json

THETA_EXT = 0.27
THETA_RET = np.pi/4


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho*np.cos(phi)
    y = rho*np.sin(phi)
    return(x, y)

def get_other_angles(theta_shoulder):
  theta_wrist = theta_shoulder + np.pi/2
  theta_elbow = np.pi/2 - 2*theta_shoulder
  return theta_wrist, theta_elbow

class BraccioXYBBTargetInterface(object):
  """BraccioXYBBTargetInterface"""
  def __init__(self):
    super(BraccioXYBBTargetInterface, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('braccio_xy_bb_target', anonymous=True)

    group_name = "braccio_arm"
    self.move_group = moveit_commander.MoveGroupCommander(group_name)

    self.gripper_group = moveit_commander.MoveGroupCommander("braccio_gripper")

    self.bounding_box = [0,0,0,0,0]
    self.subscriber = rospy.Subscriber("bounding_box",  Int16MultiArray, self.bb_callback, queue_size=1)
    self.homography = None

    self.input_image_compressed = "/pi3a/image/compressed"
    self.current_image = CompressedImage()

    self.subscriber = rospy.Subscriber(self.input_image_compressed,  CompressedImage, self.cal_im_callback, queue_size=1)
    self.mouseX = 100
    self.mouseY = 100

  def bb_callback(self, ros_data):
    self.bounding_box = ros_data.data

  def cal_im_callback(self, ros_data):
    self.current_image = ros_data

  def transform(self, x1, y1):
    if self.homography is not None:
      a = np.array([[x1, y1]], dtype='float32')
      return cv2.perspectiveTransform(a[None, :, :], self.homography)[0][0]
    else:
      raise ValueError('run or load calibration first!')

  def transform_bb(self):
    x, y = self.bounding_box_center()
    return self.transform(x,y)

  def bounding_box_center(self):
    x = (self.bounding_box[1]+self.bounding_box[3])/2
    y = (self.bounding_box[2]+self.bounding_box[4])/2
    return x, y

  def draw_circle(self,event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
      self.mouseX, self.mouseY = x,y

  def wait_for_image_click(self):
    self.mouseX, self.mouseY = None, None
    while(True):
      np_arr = np.fromstring(self.current_image.data, np.uint8)
      img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      if self.mouseX:
        cv2.circle(img,(self.mouseX, self.mouseY),10,(255,0,0),-1)
      cv2.imshow('image',img)
      k = cv2.waitKey(20) & 0xFF
      if self.mouseX or k == ord('p'):
          break
    return self.mouseX, self.mouseY

  def calibrate(self):
    cv2.namedWindow('image')
    cv2.setMouseCallback('image',self.draw_circle)
    while len(self.current_image.data) == 0:
      time.sleep(10)

    src_pts = []
    dst_angs = []
    mouseX = None
    while not mouseX:
      print 'click on robot base.'
      mouseX, mouseY = self.wait_for_image_click()
    src_pts.append([mouseX,mouseY])

    self.gripper_middle()
    N = 8
    phi_min = np.pi/6
    phi_max = np.pi - np.pi/6
    for i in range(2,N):
      print 'click on location, press p to pass.'
      self.go_to_raise()
      if i % 2 == 0:
        rand_phi = phi_min + i*(phi_max-phi_min)/N
        theta_shoulder = THETA_RET
      else:
        theta_shoulder = THETA_EXT
      theta_wrist, theta_elbow = get_other_angles(theta_shoulder)
      rand_targ = [rand_phi,theta_shoulder,theta_elbow, theta_wrist]
      self.go_to_joint(rand_targ)
      mouseX, mouseY = self.wait_for_image_click()
      if mouseX:
        src_pts.append([mouseX,mouseY])
        dst_angs.append(rand_targ)
    with open('calibration.json', 'w') as f:
      json.dump({'src_pts':src_pts,'dst_angs':dst_angs},f)
    self.load_calibrate()

  def load_calibrate(self):
    with open('calibration.json', 'r') as f:
      calib = json.load(f)
    src_pts = calib['src_pts']
    dst_angs = calib['dst_angs']

    s_ret_pts = src_pts[1::2]
    s_ext_pts = src_pts[2::2]
    arr = np.array(s_ret_pts)-np.array(s_ext_pts)
    self.L = np.sqrt((arr*arr).sum(axis=1)).mean()/(np.cos(THETA_EXT)-np.cos(THETA_RET))
    arr = np.array(s_ret_pts)-np.array(src_pts[0])
    l1 = np.sqrt((arr*arr).sum(axis=1)).mean() - self.L*np.cos(THETA_RET)
    arr = np.array(s_ext_pts)-np.array(src_pts[0])
    l2 = np.sqrt((arr*arr).sum(axis=1)).mean() - self.L*np.cos(THETA_EXT)
    self.l = (l1+l2)/2

    print 'l = ' + str(self.l)
    print 'L = ' + str(self.L)

    dst_pts = [[0,0]]
    for i in range(len(dst_angs)):
      phi = dst_angs[i][0]
      rho = self.L*np.cos(dst_angs[i][1]) + self.l
      x, y = pol2cart(rho, phi)
      dst_pts.append([x,y])

    print src_pts
    print dst_pts

    src_pts = np.array(src_pts)
    dst_pts = np.array(dst_pts)

    h, status = cv2.findHomography(src_pts, dst_pts)
    self.homography = h

    print 'calibration done. transformed_bb:'
    self.print_pose()
    cv2.destroyAllWindows()
    self.go_to_up()

  def go_to_joint(self, joint_targets):
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = joint_targets[0]
    joint_goal[1] = joint_targets[1]
    joint_goal[2] = joint_targets[2]
    joint_goal[3] = joint_targets[3]
    joint_goal[4] = 1.5708
    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()

  def gripper_close(self):
    self.go_gripper(0.9)

  def gripper_open(self):
    self.go_gripper(0.2)

  def gripper_middle(self):
    self.go_gripper(0.8)

  def go_gripper(self, val):
    joint_goal = self.gripper_group.get_current_joint_values()
    joint_goal[0] = val
    joint_goal[1] = val
    print joint_goal
    self.gripper_group.go(joint_goal, wait=True)
    self.gripper_group.stop()

  def go_to_raise(self):
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 1.15
    joint_goal[2] = 0.13
    joint_goal[3] = 2.29
    self.go_to_joint(joint_goal)

  def go_to_push(self, phi):
    self.go_to_raise()
    self.gripper_middle()
    if phi:
      joint_goal = self.move_group.get_current_joint_values()
      joint_goal[0] = float(phi)
      self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 2.7
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 2.1
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 0.5
    joint_goal[2] = 1.8
    joint_goal[3] = 0.1
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 2.1
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 2.7
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)

  def get_targets(self,x,y):
    s, phi = cart2pol(x,y)

    theta_shoulder = np.arccos((s - self.l)/self.L)
    theta_wrist, theta_elbow = get_other_angles(theta_shoulder)
    return [phi, theta_shoulder, theta_elbow, theta_wrist]

  def go_to_xy(self, x, y):
    joint_targets = self.get_targets(x,y)

    print joint_targets

    if joint_targets[1] < THETA_EXT:
      print '++++++ Not in Domain ++++++'
      print 'theta = ' + str(joint_targets[1])
      print '+++++++++++++++++++++++++++'
      return
    if joint_targets[1] > THETA_RET:
      print '++++++ Too close, pushing backwards +++++'
      print 'theta = ' + str(joint_targets[1])
      self.go_to_push(joint_targets[0])
      return

    self.go_to_raise()
    self.gripper_open()
    joint_goal = self.move_group.get_current_joint_values()

    joint_goal[0] = float(joint_targets[0])
    self.go_to_joint(joint_goal)

    joint_goal[1] = float(joint_targets[1])
    joint_goal[2] = float(joint_targets[2])
    joint_goal[3] = float(joint_targets[3])
    self.go_to_joint(joint_goal)
    self.gripper_close()
    self.go_to_home()


  def go_to_manual(self):
    print 'pos x?'
    tst = raw_input()
    if tst!='':
        x = float(tst)
    print 'pos y?'
    tst = raw_input()
    if tst!='':
        y = float(tst)

    self.go_to_xy(x, y)

  def go_to_target(self):
    v = self.transform_bb()
    print v
    self.go_to_xy(v[0], v[1])

  def go_to_home(self):

    self.go_to_raise()
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 3.14
    self.go_to_joint(joint_goal)
    self.gripper_open()

  def go_to_up(self):
    self.go_to_raise()

    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 1.5708
    joint_goal[1] = 1.5708
    joint_goal[2] = 1.5708
    joint_goal[3] = 1.5708

    self.go_to_joint(joint_goal)

  def print_pose(self):
    print self.gripper_group.get_current_joint_values()
    x, y = self.bounding_box_center()
    print x, y

def main():
  bb_targetter = BraccioXYBBTargetInterface()

  while True:
      print "============ instructions: p=print, h=home, u=up, c=calibrate, l=load_calibration, t=target, m=manual, q=quit"
      inp = raw_input()
      if inp=='q':
          break
      if inp=='p':
          bb_targetter.print_pose()
      if inp=='c':
          bb_targetter.calibrate()
      if inp=='l':
          bb_targetter.load_calibrate()
      if inp=='t':
          bb_targetter.go_to_target()
      if inp=='m':
          bb_targetter.go_to_manual()
      if inp=='h':
          bb_targetter.go_to_home()
      if inp=='u':
          bb_targetter.go_to_up()


if __name__ == '__main__':
  main()
