#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import manip
color_infos=(0, 255, 255)
#--- Define our Class

# tourne = 0
global tourne

class image_converter:
    
  def __init__(self):
    #--- Subscriber to the camera flow
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.tourne = 2
    # self.arm=manip.MoveArm()
    # self.arm.go_to_home_pose()

  def callback(self,data):  #--- Callback function
    # global tourne
    # print(self.tourne)
    # try:
    #   a=tourne
    # except NameError:
    #   global tourne
    #   tourne=2
    # try:
    #   Afficher = mes
    # except:
    #   Afficher=True
    #          Conversion de l'image via cv_bridge
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    #          Changement d'espace colorimetrique de BGR a HSVx	
    hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    low_blue = np.array([94, 80, 2])
    high_blue = np.array([126, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
    blue = cv2.bitwise_and(cv_image, cv_image, mask=blue_mask)
    blue_resize = cv2.resize(blue, (360, 240)) 
    image_resize = cv2.resize(cv_image, (360, 240)) 
    elementsR=cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(elementsR) > 0:
      c=max(elementsR, key=cv2.contourArea)
      ((x, y), rayon)=cv2.minEnclosingCircle(c)
      cv2.circle(blue, (int(x), int(y)), int(rayon), color_infos, 2)
      cv2.circle(cv_image, (int(x), int(y)), 5, color_infos, 10)
      cv2.line(cv_image, (int(x), int(y)), (int(x)+150, int(y)), color_infos, 2)
      cv2.putText(cv_image, "", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 0.4, color_infos, 1, cv2.LINE_AA)
      cv2.circle(blue, (int(x), int(y)), 1, color_infos, 1)
      hauteur, longeur = blue.shape[:2]
      cv2.circle(blue, (int(longeur/2),int(hauteur/2)),5,color_infos, 5)
      cv2.line(blue, (int(x), int(y)), (int(longeur/2),int(hauteur/2)), color_infos, 2)
      Var_color=(0, max(255, 255-10*int(abs(x-longeur/2)/2)), min(255, 10*int(abs(x-longeur/2)/2)))
      if (int(x)-(longeur/2)) > 5 :
        self.tourne = -1
        # cv2.arrowedLine(cv_image, (int(longeur/2),int(hauteur/2)),(int(x), int(hauteur/2)),Var_color, 1, tipLength=0.4)
        # on tourne a droite
      elif (int(x)-(longeur/2)) < -5 :
        self.tourne = 1
        # cv2.arrowedLine(cv_image, (int(longeur/2),int(hauteur/2)), (int(x), int(hauteur/2)),Var_color, 1, tipLength=0.4)
        # on tourne a gauche
      else:
        # cv2.putText(cv_image, "OK", (int(longeur/2),int(hauteur/2)), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 255, 0), 1)
        self.tourne = 0
        # C'est au millieu
      
    else:
      self.tourne=-2
      # on cherche l'objet !

    cv2.namedWindow("Masque bleu")
    cv2.moveWindow("Masque bleu", 0, 0)
    cv2.imshow("Masque bleu", blue_resize)
    cv2.namedWindow("Image camera")
    cv2.moveWindow("Image camera", 0, 400)
    cv2.imshow("Image camera", image_resize)
    cv2.waitKey(1)
  
    #attendre une touche
    key = cv2.waitKey(30)&0xFF
    if key == ord('q'):
      # print("q")	
      sys.exit()
      cv2.destroyAllWindows()
    if key == ord('r'):
      self.reset()
      # print("reset!")

  def reset(self):
    rospy.wait_for_service("/gazebo/reset_world")
    reset_wold = rospy.ServiceProxy("/gazebo/reset_world",Empty)
    reset_wold()

class laser_distance:

  def __init__(self, ic):
    self.laser_sub=rospy.Subscriber('/scan',LaserScan,self.callback)
    self.ic = ic
  
  def callback(self,msg):
    if self.ic.tourne == 1:
      if msg.ranges[0] > 0.16 and msg.ranges[1] > 0.18 and msg.ranges[2] > 0.20 and msg.ranges[3] > 0.22 and msg.ranges[23] > 0.22 and msg.ranges[22] > 0.20 and msg.ranges[21] > 0.18 and msg.ranges[20] > 0.16:
        self.avancer_gauche()
      else :
        self.tourne_gauche()
    elif self.ic.tourne == -1:
      if msg.ranges[0] > 0.16 and msg.ranges[1] > 0.18 and msg.ranges[2] > 0.20 and msg.ranges[3] > 0.22 and msg.ranges[23] > 0.22 and msg.ranges[22] > 0.20 and msg.ranges[21] > 0.18 and msg.ranges[20] > 0.16:
        self.avancer_droite()
      else :
        self.tourne_droite()
    elif self.ic.tourne == 0:
      if msg.ranges[0] > 0.16 and msg.ranges[1] > 0.18 and msg.ranges[2] > 0.20 and msg.ranges[3] > 0.22 and msg.ranges[23] > 0.22 and msg.ranges[22] > 0.20 and msg.ranges[21] > 0.18 and msg.ranges[20] > 0.16:
        self.avancer()
        # print("Avancer !!")
      else:
        self.areter()
        # print("Stop !!!")
            
    else:
      self.rapide_tourne_droite()
      print("Recherche Objet Bleu")

  def avancer_gauche(self):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #avancer
    speed = Twist()    
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0.05
    speed.linear.x = 0.2
    pub.publish(speed)
  def avancer_droite(self):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #avancer
    speed = Twist()    
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = -0.05
    speed.linear.x = 0.2
    pub.publish(speed)

  def avancer(self):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #avancer
    speed = Twist()    
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0
    speed.linear.x = 0.2
    pub.publish(speed)
  def areter(self):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #areter
    speed = Twist()    
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0
    pub.publish(speed)
  def tourne_droite(self):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #areter
    speed = Twist()    
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = -0.05
    pub.publish(speed)
  def rapide_tourne_droite(self):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #areter
    speed = Twist()    
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = -1
    pub.publish(speed)
  def tourne_gauche(self):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #areter
    speed = Twist()    
    speed.linear.x = 0
    # speed.linear.y = manip
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0.05
    pub.publish(speed)



#--------------- MAIN LOOP
def main(args):
    # print("En cour!")
    #--- Initialize the ROS node
    rospy.init_node('image_converter', anonymous=True)
    #--- Create the object from the class we defined before
    ic = image_converter()
    laser_distance(ic)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    #--- In the end remember to close all cv windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)
