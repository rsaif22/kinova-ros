#! /usr/bin/env python3
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from geometry_msgs.msg import Vector3
import argparse
import time
import numpy as np

def argumentParser(argument):
  """ Argument parser """
  parser = argparse.ArgumentParser(description='Drive robot joint to command position')
  parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                    help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
  #args_ = parser.parse_args(argument)
  argv = rospy.myargv()
  args_ = parser.parse_args(argv[1:])
  prefix = args_.kinova_robotType
  nbJoints = int(args_.kinova_robotType[3])	
  nbfingers = int(args_.kinova_robotType[5])	
  return prefix, nbJoints, nbfingers

def moveJoint (jointcmds,prefix,nbJoints):
  topic_name = '/' + prefix + '/effort_joint_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 50):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()     

def moveFingers (jointcmds,prefix,nbJoints):
  topic_name = '/' + prefix + '/effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)  
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_finger_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()     

def disable_gravity():

    # Disable gravity
    set_physics_properties = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
    get_physics_properties = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)

    # Get the current physics properties
    physics_properties = get_physics_properties()

    # Store the original gravity value
    original_gravity = physics_properties.gravity

    # Set gravity to 0
    zero_gravity = Vector3()
    print(f"Gravity is {zero_gravity}")
    physics_properties.gravity = zero_gravity
    #print(f"TIME STEP IS {physics_properties.time_step}")
    print(f"MAX UPDATE RATE IS {physics_properties.max_update_rate}")

    # Call the service to disable gravity
    set_physics_properties(physics_properties.time_step, physics_properties.max_update_rate,
                           physics_properties.gravity, physics_properties.ode_config)
    
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo()

    # Wait for the controllers to start
    rospy.wait_for_service('/j2n6s300/controller_manager/load_controller')  # Replace with your controller service
    rospy.sleep(3)
    # Enable gravity again
    for i in range(10):
      physics_properties.gravity.z = physics_properties.gravity.z + original_gravity.z/10
      print(f"Gravity is {physics_properties.gravity}")

    # Call the service to enable gravity
      set_physics_properties(physics_properties.time_step, physics_properties.max_update_rate,
                           physics_properties.gravity, physics_properties.ode_config)
      rospy.sleep(0.1)

if __name__ == '__main__':
  try:    
    rospy.init_node('move_robot_using_trajectory_msg')		
    prefix, nbJoints, nbfingers = argumentParser(None)    
    #allow gazebo to launch
    time.sleep(5)

    # Unpause the physics
    # Wait for the controller services to become available
    disable_gravity()



    # if (nbJoints==6):
    #   #home robots
    #   moveJoint ([0,np.pi/2,np.pi,np.pi/2,0.0,0.0],prefix,nbJoints)
    # else:
    #   moveJoint ([0.0,2.9,0.0,1.3,4.2,1.4,0.0],prefix,nbJoints)

    #moveFingers ([1,1,1],prefix,nbfingers)
  except rospy.ROSInterruptException:
    print("program interrupted before completion")
