#!/usr/bin/env python3
import sys
import rospy
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list





Objectif=[0.2,0.0,0.163]
Objectif2=[0.201,0.006,0.298]
init_pose = [0,0,0,0]
home_pose = [0,-1,0.3,0.7]



def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a	 Pose or a PoseStamped
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

class MoveArm(object):
  """MoveArm"""
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    robot = moveit_commander.RobotCommander()


    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    group_names = robot.get_group_names()

    # Misc variables
    self.robot = robot
    self.group = group
    self.group_names = group_names
    self.display_trajectory_publisher = display_trajectory_publisher

  def use_joint(self,Goal):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = Goal
    group.go(joint_goal, wait=True)
    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_home_pose(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = home_pose

    group.go(joint_goal, wait=True)
    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_init_pose(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = init_pose

    group.go(joint_goal, wait=True)
    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_coordonne(self,Goal):

    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = Goal[0]
    pose_goal.position.y = Goal[1]
    pose_goal.position.z = Goal[2]
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)

    group.stop()

    group.clear_pose_targets()


    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def move_x(self,Distance):
#Distance < 0 sa va en arriere  et si >0 sa va devant

    group = self.group
    cartesian_plan, fraction = self.plan_cartesian_path(Distance,0)
    self.display_trajectory(cartesian_plan)
    self.execute_plan(cartesian_plan)

    group.clear_pose_targets()
    return True

  def move_y(self,Distance):
#Distance < 0 sa va en arriere  et si >0 sa va devant

    group = self.group
    cartesian_plan, fraction = self.plan_cartesian_path(Distance,1)
    self.display_trajectory(cartesian_plan)
    self.execute_plan(cartesian_plan)

    group.clear_pose_targets()
    return True

  def move_z(self,Distance): # Ne marche tout simplement pas Je c pas pk
#Distance < 0 sa va en arriere  et si >0 sa va devant

    group = self.group
    cartesian_plan, fraction = self.plan_cartesian_path(Distance,2)
    self.display_trajectory(cartesian_plan)
    self.execute_plan(cartesian_plan)

    group.clear_pose_targets()
    return True



  def plan_cartesian_path(self, Distance , coordonne):
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    if coordonne == 0:
      wpose.position.x += Distance  #Suivant x
    elif coordonne == 1:
      wpose.position.y += Distance  #Suivant y
    elif coordonne == 2:
      wpose.position.z += Distance  #Suivant z

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory);

  def execute_plan(self, plan):
    group = self.group
    group.execute(plan, wait=True)




  

def main():
  try:
    tutorial = MoveArm()

    print("============ Press `Enter` to execute a movement to init pose ...")
    raw_input()
    tutorial.go_to_init_pose()
    print("============ X")
    raw_input()
    tutorial.move_x(0.02)
    tutorial.move_x(0.02)
    tutorial.move_x(0.02)
    print("============ Z")
    raw_input()
    tutorial.move_z(0.02)
    print("============ Z2 , 1")
    raw_input()
    tutorial.move_z(0.02)
    print("============ Z2 , -1")
    raw_input()
    tutorial.move_z(-0.02)
    print("============ Y")
    raw_input()
    tutorial.move_y(0.02)
    print("============ Z2, 2")
    raw_input()
    tutorial.move_z(0.02)
    tutorial.move_z(-0.02)
    tutorial.move_x(-0.02)
    tutorial.move_x(-0.02)
    tutorial.move_x(-0.02)
    tutorial.move_x(-0.02)
    tutorial.move_x(-0.02)

    print("============ Press `Enter` to execute a movement to init pose ...")
    raw_input()
    tutorial.go_to_init_pose()



    print("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
