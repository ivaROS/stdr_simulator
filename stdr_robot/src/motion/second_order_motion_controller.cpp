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
The motion is decomposed into two translations alond the x axis of the
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
    // _currentVel = geometry_msgs::Twist();
    // acc_publisher = n.advertise<geometry_msgs::Twist>(name + "/acc", 1000);
    //vel_publisher = n.advertise<geometry_msgs::Twist>(name + "/current_vel", 1000);

    prev_error_x = 0.0;
    prev_error_y = 0.0;
    prev_error_theta = 0.0;

    K_p_x = 3.0;
    K_p_y = 3.0;
    K_p_z = 3.0;

    K_d_x = 0.00;
    K_d_y = 0.00;
    K_d_z = 0.00;
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
    // ROS_INFO("~~~~~~~in second order dynamics~~~~~~~~~");
    // Simple omni model
    // TODO: Add kinematic model uncertainties
    // _currentTwist is the desired velocity
    // current_vel is the current velocity
    // std::cout << "in dynamics" << std::endl;
    //
    if ( _currentVel.angular.z != 0 || _currentVel.linear.x != 0 || _currentVel.linear.y != 0 || 
      _currentTwist.angular.z != 0 || _currentTwist.linear.x != 0 || _currentTwist.linear.y != 0) 
    {
      // Dx and Dy takes under consideration both linear rotations, 
      // independently of each other
      // Kp: 1, Kd: .05 (div by 20)
      std::cout << "current twist command: " << _currentTwist.linear.x << ", " << _currentTwist.linear.y << ", " << _currentTwist.angular.z << std::endl;
      std::cout << "current robot velocity: " << _currentVel.linear.x << ", " << _currentVel.linear.y << ", " << _currentVel.angular.z << std::endl;
      
      double error_x = _currentTwist.linear.x - _currentVel.linear.x;
      double error_y = _currentTwist.linear.y - _currentVel.linear.y;
      double error_theta = _currentTwist.angular.z - _currentVel.angular.z;

      double d_error_x_dt = (error_x - prev_error_x) / dt.toSec();
      double d_error_y_dt = (error_y - prev_error_y) / dt.toSec();
      double d_error_theta_dt = (error_theta - prev_error_theta) / dt.toSec();

      double a_x = K_p_x * error_x + K_d_x * d_error_x_dt;
      double a_y = K_p_y * error_y + K_d_y * d_error_y_dt;
      double a_theta = K_p_z * error_theta + K_d_z * d_error_theta_dt;
      std::cout << "raw robot acceleration: " << a_x << ", " << a_y << ", " << a_theta << std::endl;

      double linear_acc_lim = 3.0;
      double angular_acc_lim = 3.0;

      _currentAcc.linear.x = (a_x > 0.0) ? std::min(a_x, linear_acc_lim) : std::max(a_x, -linear_acc_lim);
      _currentAcc.linear.y = (a_y > 0.0) ? std::min(a_y, linear_acc_lim) : std::max(a_y, -linear_acc_lim);
      _currentAcc.angular.z = (a_theta > 0.0) ? std::min(a_theta, angular_acc_lim) : std::max(a_theta, -angular_acc_lim);      
      std::cout << "clipped robot acceleration: " << _currentAcc.linear.x << ", " << _currentAcc.linear.y << ", " << _currentAcc.angular.z << std::endl;

      _pose.x += 
        _currentVel.linear.x * dt.toSec() * cosf(_pose.theta) + 
        _currentVel.linear.y * dt.toSec() * cosf(_pose.theta + M_PI/2.0); 

      _pose.y += 
        _currentVel.linear.y * dt.toSec() * sinf(_pose.theta + M_PI/2.0) +
        _currentVel.linear.x * dt.toSec() * sinf(_pose.theta);

      _pose.theta += _currentVel.angular.z * dt.toSec();

      _currentVel.linear.x += _currentAcc.linear.x * dt.toSec();
      _currentVel.linear.y += _currentAcc.linear.y * dt.toSec();
      _currentVel.angular.z += _currentAcc.angular.z * dt.toSec();
      // vel_publisher.publish(_currentVel);
      // pretty sure pose is in world coords, vel/acc are in robot coords

      prev_error_x = error_x;
      prev_error_y = error_y;
      prev_error_theta = error_theta;
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
