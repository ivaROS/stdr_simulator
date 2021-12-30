/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
   
Authors:

Oscar Lima Carrion, olima_84@yahoo.com
Ashok Meenakshi, mashoksc@gmail.com
Seguey Alexandrov, sergeyalexandrov@mail.com

Review / Edits:
Manos Tsardoulias, etsardou@gmail.com

About this code:

This class represents a motion model for omnidirectional robot and could be
used to sample the possible pose given the starting pose and the commanded
robot's motion.
T
he motion is decomposed into two translations alond the x axis of the
robot (forward), and along the y axis of the robot (lateral), and one
rotation.
******************************************************************************/

#include <stdr_robot/motion/second_order_motion_controller.h>

namespace stdr_robot {
    
  /**
  @brief Default constructor
  @param pose [const geometry_msgs::Pose2D&] The robot pose
  @param tf [tf::TransformBroadcaster&] A ROS tf broadcaster
  @param n [ros::NodeHandle&] The ROS node handle
  @param name [const std::string&] The robot frame id
  @return void
  **/  
  SecondOrderMotionController::SecondOrderMotionController(
    const geometry_msgs::Pose2D& pose, 
    tf::TransformBroadcaster& tf, 
    ros::NodeHandle& n, 
    const std::string& name,
    const stdr_msgs::KinematicMsg params)
      : MotionController(pose, tf, name, n, params)
  {
    _calcTimer = n.createTimer(
      _freq, 
      &SecondOrderMotionController::calculateMotion, 
      this);
    current_vel = geometry_msgs::Twist();
    acc_publisher = n.advertise<sensor_msgs::Imu>(name + "/imu", 1000);
    current_acc = sensor_msgs::Imu();
  }

  
  /**
  @brief Calculates the motion - updates the robot pose
  @param event [const ros::TimerEvent&] A ROS timer event
  @return void
  **/
  void SecondOrderMotionController::calculateMotion(const ros::TimerEvent& event) 
  {
    //!< updates _posePtr based on _currentTwist and time passed (event.last_real)
    
    ros::Duration dt = ros::Time::now() - event.last_real;
    ROS_INFO("~~~~~~~in second order dynamics~~~~~~~~~");
    // Simple omni model
    // TODO: Add kinematic model uncertainties
    if (_currentTwist.angular.z != 0 || _currentTwist.linear.x != 0 ||
     _currentTwist.linear.y != 0) 
    {
      // Dx and Dy takes under consideration both linear rotations, 
      // independently of each other
      double K_acc = 3.0;
      double a_x = -K_acc * (current_vel.linear.x - _currentTwist.linear.x);
      double a_y = -K_acc * (current_vel.linear.y - _currentTwist.linear.y);
      double a_theta = -K_acc * (current_vel.angular.z - _currentTwist.angular.z);

      current_acc.linear_acceleration.x = a_x;
      current_acc.linear_acceleration.y = a_y;
      current_acc.angular_velocity.z = a_theta;

      acc_publisher.publish(current_acc);
      _pose.x += 
        current_vel.linear.x * dt.toSec() * cosf(_pose.theta) + 
        current_vel.linear.y * dt.toSec() * cosf(_pose.theta + M_PI/2.0); 

      _pose.y += 
        current_vel.linear.y * dt.toSec() * sinf(_pose.theta + M_PI/2.0) +
        current_vel.linear.x * dt.toSec() * sinf(_pose.theta);

      _pose.theta += current_vel.angular.z * dt.toSec();

      current_vel.linear.x += a_x * dt.toSec();
      current_vel.linear.y += a_y * dt.toSec();
      current_vel.angular.z += a_theta * dt.toSec();
    }
  }
  
  /**
  @brief Default destructor 
  @return void
  **/
  SecondOrderMotionController::~SecondOrderMotionController(void)
  {
    
  }
    
}
