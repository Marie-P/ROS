import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
  print('=====================================')
  print('s1 [270]')
  print(msg.ranges[270])
  print('s3 [0]')
  print(msg.ranges[0])

  print('s3 [90]')
  print(msg.ranges[90])

  if msg.ranges[0] > 0.5: # nord
    move.linear.x = 0.5
    move.angular.z = 0.0
  elif msg.ranges[270] > 0.5: # ouest
    move.linear.x = 0.0
    move.angular.z = -0.5
  elif msg.ranges[180] > 0.5: # sud
    move.linear.x = -0.5
    move.angular.z = 0.0
  elif msg.ranges[90] > 0.5: # est
    move.linear.x = 0.0
    move.angular.z = 0.5
  else:
    move.linear.x = 0.0
    move.angular.z = 0.0
  
  pub.publish(move)

rospy.init_node('obstacle_avoidance')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist)
move = Twist()

rospy.spin()