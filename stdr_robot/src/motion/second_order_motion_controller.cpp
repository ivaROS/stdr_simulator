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
    ROS_INFO_STREAM(_namespace << ", 2nd order controller with frequency: " << _freq);

    _calcTimer = n.createTimer(
      _freq, 
      &SecondOrderMotionController::calculateMotion, 
      this);

    error_x_kmin1 = 0.0;
    error_y_kmin1 = 0.0;
    error_theta_kmin1 = 0.0;

    K_p_x = 5.0;
    K_p_y = 5.0;
    K_p_z = 5.0;

    K_d_x = 0.1;
    K_d_y = 0.1;
    K_d_z = 0.1;

    // K_i_x = 50.0;
    // K_i_y = 50.0;
    // K_i_z = 50.0;    
  }

  
  /**
  @brief Calculates the motion - updates the robot pose
  @param event [const ros::TimerEvent&] A ROS timer event
  @return void
  **/
  void SecondOrderMotionController::calculateMotion(const ros::TimerEvent& event) 
  {
    //!< updates _posePtr based on _currentCommandVel and time passed (event.last_real)
    
    ros::Duration timestep = ros::Time::now() - event.last_real;
    // ROS_INFO("~~~~~~~in second order dynamics~~~~~~~~~");
    // Simple omni model
    // TODO: Add kinematic model uncertainties
    // _currentCommandVel is the desired velocity
    // current_vel is the current velocity
    // std::cout << "in dynamics" << std::endl;
    //
    if ( _currentVel.angular.z != 0 || _currentVel.linear.x != 0 || _currentVel.linear.y != 0 || 
         _currentCommandVel.angular.z != 0 || _currentCommandVel.linear.x != 0 || _currentCommandVel.linear.y != 0) 
    {
      // Dx and Dy takes under consideration both linear rotations, 
      // independently of each other
      // Kp: 1, Kd: .05 (div by 20)
      // ROS_INFO_STREAM("current twist command: " << _currentCommandVel.linear.x << ", " << _currentCommandVel.linear.y << ", " << _currentCommandVel.angular.z);
      // ROS_INFO_STREAM("current robot velocity: " << _currentVel.linear.x << ", " << _currentVel.linear.y << ", " << _currentVel.angular.z);
      
      double dt = timestep.toSec();

      // Proportional control
      double error_x = _currentCommandVel.linear.x - _currentVel.linear.x;
      double error_y = _currentCommandVel.linear.y - _currentVel.linear.y;
      double error_theta = _currentCommandVel.angular.z - _currentVel.angular.z;

      // Derivative control
      d_error_x_dt_k = (error_x - error_x_kmin1) / dt;
      d_error_y_dt_k = (error_y - error_y_kmin1) / dt;
      d_error_theta_dt_k = (error_theta - error_theta_kmin1) / dt;

      double d_error_x_dt = (d_error_x_dt_k + d_error_x_dt_kmin1 + d_error_x_dt_kmin2) / 3;
      double d_error_y_dt = (d_error_y_dt_k + d_error_y_dt_kmin1 + d_error_y_dt_kmin2) / 3;
      double d_error_theta_dt = (d_error_theta_dt_k + d_error_theta_dt_kmin1 + d_error_theta_dt_kmin2) / 3;

      // Integral control
      // double int_error_x_k = int_error_x_kmin1 + 0.5 * (error_x + error_x_kmin1) * dt;
      // double int_error_y_k = int_error_y_kmin1 + 0.5 * (error_y + error_y_kmin1) * dt;
      // double int_error_theta_k = int_error_theta_kmin1 + 0.5 * (error_theta + error_theta_kmin1) * dt;

      // ROS_INFO_STREAM("error_x: " << error_x << ", error_y: " << error_y << ", error_theta: " << error_theta);
      // ROS_INFO_STREAM("d_error_x_dt_k: " << d_error_x_dt_k << ", d_error_y_dt_k: " << d_error_y_dt_k << ", d_error_theta_dt_k: " << d_error_theta_dt_k);
      double a_x = K_p_x*error_x + K_d_x * d_error_x_dt; //  + K_i_x * int_error_x_k;
      double a_y = K_p_y*error_y  + K_d_y * d_error_y_dt; //  + K_i_y * int_error_y_k;
      double a_theta = K_p_z*error_theta + K_d_z * d_error_theta_dt; //  + K_i_z * int_error_theta_k;
      // ROS_INFO_STREAM("raw robot acceleration: " << a_x << ", " << a_y << ", " << a_theta);

      double linear_acc_lim = 2.5;
      double angular_acc_lim = 2.5;

      _currentAcc.twist.linear.x = std::max(-linear_acc_lim, std::min(linear_acc_lim, a_x));
      _currentAcc.twist.linear.y = std::max(-linear_acc_lim, std::min(linear_acc_lim, a_y));
      _currentAcc.twist.angular.z = std::max(-angular_acc_lim, std::min(angular_acc_lim, a_theta));      
      // ROS_INFO_STREAM("clipped robot acceleration: " << _currentAcc.linear.x << ", " << _currentAcc.linear.y << ", " << _currentAcc.angular.z);

      _pose.x += 
        (_currentVel.linear.x * dt * cosf(_pose.theta) - 
         _currentVel.linear.y * dt * sinf(_pose.theta) ); 

      // ^^^ was previously cosf(_pose.theta + M_PI/2.0)

      _pose.y += 
        (_currentVel.linear.y * dt * cosf(_pose.theta) +
         _currentVel.linear.x * dt * sinf(_pose.theta));

      _pose.theta += _currentVel.angular.z * dt;

      _currentVel.linear.x += (_currentAcc.twist.linear.x * dt );
      _currentVel.linear.y += (_currentAcc.twist.linear.y * dt );
      _currentVel.angular.z += (_currentAcc.twist.angular.z * dt );

      // pose is in world coords, vel/acc are in robot coords

      d_error_x_dt_kmin2 = d_error_x_dt_kmin1;
      d_error_x_dt_kmin1 = d_error_x_dt_k;

      d_error_y_dt_kmin2 = d_error_y_dt_kmin1;
      d_error_y_dt_kmin1 = d_error_y_dt_k;

      d_error_theta_dt_kmin2 = d_error_theta_dt_kmin1;
      d_error_theta_dt_kmin1 = d_error_theta_dt_k;            

      error_x_kmin1 = error_x;
      error_y_kmin1 = error_y;
      error_theta_kmin1 = error_theta;

      // int_error_x_kmin1 = int_error_x_k;
      // int_error_y_kmin1 = int_error_y_k;
      // int_error_theta_kmin1 = int_error_theta_k;
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
