#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char**argv){
  ros::init(argc, argv, "vel_publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Rate rate(2);
  while(ros::ok()){
    geometry_msgs::Twist move;
    move.linear.x = 1;
    move.angular.z = 1;
    pub.publish(move);
    rate.sleep();
  }
  ros::spin();
  return 0;
}
