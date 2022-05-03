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
    _currentVel = geometry_msgs::Twist();
    acc_publisher = n.advertise<sensor_msgs::Imu>(name + "/imu", 1000);
    vel_publisher = n.advertise<geometry_msgs::Twist>(name + "/current_vel", 1000);
    current_acc = sensor_msgs::Imu();

    prev_error_x = 0.0;
    prev_error_y = 0.0;
    prev_error_theta = 0.0;

    K_p = 3;
    K_d = 0.5;
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
    if (_currentTwist.angular.z != 0 || _currentTwist.linear.x != 0 || _currentTwist.linear.y != 0) 
    {
      // Dx and Dy takes under consideration both linear rotations, 
      // independently of each other
      // Kp: 1, Kd: .05 (div by 20)
      double error_x = _currentTwist.linear.x - _currentVel.linear.x;
      double error_y = _currentTwist.linear.y - _currentVel.linear.y;
      double error_theta = _currentTwist.angular.z - _currentVel.angular.z;

      //double d_error_x_dt = (error_x - prev_error_x) / dt.toSec();
      //double d_error_y_dt = (error_y - prev_error_y) / dt.toSec();
      //double d_error_theta_dt = (error_theta - prev_error_theta) / dt.toSec();

      double a_x = K_p * error_x; // + K_d*d_error_x_dt;
      double a_y = K_p * error_y; // + K_d*d_error_y_dt;
      double a_theta = K_p * error_theta; // + K_d*d_error_theta_dt;

      current_acc.linear_acceleration.x = a_x;
      current_acc.linear_acceleration.y = a_y;
      current_acc.angular_velocity.z = a_theta;

      acc_publisher.publish(current_acc);
      _pose.x += 
        _currentVel.linear.x * dt.toSec() * cosf(_pose.theta) + 
        _currentVel.linear.y * dt.toSec() * cosf(_pose.theta + M_PI/2.0); 

      _pose.y += 
        _currentVel.linear.y * dt.toSec() * sinf(_pose.theta + M_PI/2.0) +
        _currentVel.linear.x * dt.toSec() * sinf(_pose.theta);


      _pose.theta += _currentVel.angular.z * dt.toSec();

      _currentVel.linear.x += a_x * dt.toSec();
      _currentVel.linear.y += a_y * dt.toSec();
      _currentVel.angular.z += a_theta * dt.toSec();
      vel_publisher.publish(_currentVel);
      // pretty sure pose is in world coords, vel/acc are in robot coords
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
