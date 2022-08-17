#!/usr/bin/env python
from ctypes import resize
from pickle import FALSE
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

color_infos=(0, 255, 255)

class Robot:
  def __init__(self):
    self.isReset = False
    self.move = Twist()
    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.direction = 3

  def reset(self):
    print("reset")
    self.isReset = True 
    self.move.linear.x = self.move.angular.z = 0
    self.pub.publish(self.move)

  def set_direction(self, movement):
    self.move.linear.x = self.move.linear.y = self.move.linear.z = self.move.angular.x = self.move.angular.y = self.move.angular.z = 0
    if movement == 0 : # tout droit
      self.move.linear.x = 0.2
    elif movement == -1: # devant, droite
      self.move.angular.z = -0.05
      self.move.linear.x = 0.2
    elif movement == -2: # tout à droite
      self.move.angular.z = -0.05
    elif movement == 1: # devant, gauche
      self.move.angular.z = 0.05
      self.move.linear.x = 0.2
    elif movement == 2: # tout à gauche
      self.move.angular.z = 0.05
    elif movement == 3: # cherche l'objet en tournant à droite
      self.move.angular.z = -1
    self.pub.publish(self.move)

class Image_Robot:
  def __init__(self, robot):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.robot = robot

  def callback(self,data):
    try:
      self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      image_resize = cv2.resize(self.image, (360, 240))
      pink_mask, pink, pink_resize = self.createMask(self.image, (0,50,50), (10,255,255))
      contours=cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
      self.robot.direction = self.find_object(self.image, contours, pink)
      self.displayImage(pink_resize, "Masque rose", 0, 0)
      self.displayImage(image_resize, "Image camera", 0, 400)
      cv2.waitKey(1) 
      
      #attendre une touche
      k = cv2.waitKey(30)&0xFF
      if k == ord('r'):
        self.robot.reset()
      elif k == ord('g'):
        print("Go")
        self.robot.isReset = False
        self.robot.move.linear.x = 1
        self.robot.move.angular.z = 1
        self.robot.pub.publish(self.robot.move)
    except CvBridgeError as e:
      print(e)

  def createMask(self, image, low_color, high_color):
    # Changement d'espace colorimetrique de BGR a HSVx	
    hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    color_mask = cv2.inRange(hsv_frame, low_color, high_color)
    color = cv2.bitwise_and(image, image, mask=color_mask)
    image_resize = cv2.resize(color, (360, 240)) 
    return color_mask, color, image_resize

  def find_object(self, image, contours, color):
    if len(contours) > 0:
      contour=max(contours, key=cv2.contourArea)
      ((x, y), rayon)=cv2.minEnclosingCircle(contour)
      height, width = color.shape[:2]
      Var_color=(0, max(255, 255-10*int(abs(x-width/2)/2)), min(255, 10*int(abs(x-width/2)/2)))
      if (int(x)-(width/2)) > 5 : # à droite
        cv2.arrowedLine(image, (int(width/2),int(height/2)),(int(x), int(height/2)),Var_color, 1, tipLength=0.4)
        return -1
      elif (int(x)-(width/2)) < -5 : # à gauche
        cv2.arrowedLine(image, (int(width/2),int(height/2)), (int(x), int(height/2)),Var_color, 1, tipLength=0.4)
        return 1
      else: # tout droit
        return 0
    else: # recherche de l'objet
      return 3

  def displayImage(self, image, title, x, y):
    cv2.namedWindow(title)
    cv2.moveWindow(title, x, y)
    cv2.imshow(title, image)

class Laser_robot:
  def __init__(self, robot):
    self.laser_sub=rospy.Subscriber('/scan',LaserScan,self.callback)
    self.robot = robot

  def is_in_front(self, msg):
    if msg.ranges[0] > 0.16 and msg.ranges[1] > 0.18 and msg.ranges[2] > 0.20 and msg.ranges[3] > 0.22 and msg.ranges[23] > 0.22 and msg.ranges[22] > 0.20 and msg.ranges[21] > 0.18 and msg.ranges[20] > 0.16:
      return True
    return False

  def check_direction(self, msg, direction):
    if self.is_in_front(msg):
      self.robot.set_direction(direction)
    else:
      self.robot.set_direction(direction * 2)

  def callback(self,msg): 
    if self.robot.direction < 3:
      self.check_direction(msg, self.robot.direction)        
    else:
      self.robot.set_direction(3)
      print("Recherche Objet Rose")

if __name__ == '__main__':
  try:
    rospy.init_node('robot_cleaner', anonymous=True)
    robot_cleaner = Robot()
    camera = Image_Robot(robot_cleaner)
    Laser_robot(robot_cleaner)
    rospy.spin()
  except rospy.ROSInterruptException:
      pass