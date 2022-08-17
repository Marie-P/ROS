#!/usr/bin/env python3

import rospy
# import manip
# import laser
from sensor_msgs.msg import LaserScan
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
# import time



def callback(msg):
  print("Laser bras :", msg.ranges[0])
  rospy.wait_for_service('/goal_joint_space_path')
  goal_joint_space_path_service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
  goal_joint_space_path_request_object = SetJointPositionRequest()

  # 1st Movement
  goal_joint_space_path_request_object.planning_group = 'arm'
  goal_joint_space_path_request_object.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
  # goal_joint_space_path_request_object.joint_position.position = [0.0, -1, 0.3, 0.7]
  goal_joint_space_path_request_object.joint_position.position = [0.0, 0.5, 0.0, 0.0]
  goal_joint_space_path_request_object.joint_position.max_accelerations_scaling_factor = 1.0
  goal_joint_space_path_request_object.joint_position.max_velocity_scaling_factor = 1.0
  goal_joint_space_path_request_object.path_time = 2.0
  result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)

  # print(msg.data)


def callback2(msg):
  print("Laser robot :", msg.ranges[0])
  

if __name__=="__main__":
  rospy.init_node('Mouvement')
  laserBras_sub=rospy.Subscriber('/scan2',LaserScan,callback)
  laserRobot_sub=rospy.Subscriber('/scan',LaserScan,callback2)
  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Shutting down")