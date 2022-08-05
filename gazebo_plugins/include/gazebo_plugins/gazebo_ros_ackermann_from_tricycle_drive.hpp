// Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_ACKERMANN_FROM_TRICYCLE_DRIVE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_ACKERMANN_FROM_TRICYCLE_DRIVE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosAckermannDrive2Private;

/// A tricycle drive plugin for gazebo.
/**
  Example Usage:
  \code{.xml}

  <plugin name='gazebo_ros_ackermann_from_tricycle_drive' filename='libgazebo_ros_ackermann_from_tricycle_drive.so'>

    <ros>
      <namespace></namespace>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>

    <update_rate>10.0</update_rate>

    <publish_odom>false</publish_odom>
    <publish_odom_tf>false</publish_odom_tf>


    <steering_joint>back_steering_joint</steering_joint>
    <actuated_wheel_joint>back_wheel_joint</actuated_wheel_joint>

    <right_steering_joint>back_right_wheel_steering_joint</right_steering_joint>
    <actuated_right_wheel_joint>back_right_wheel_joint</actuated_right_wheel_joint>

    <left_steering_joint>back_left_wheel_steering_joint</left_steering_joint>
    <actuated_left_wheel_joint>back_left_wheel_joint</actuated_left_wheel_joint>

    <encoder_wheel_left_joint>front_left_wheel_joint</encoder_wheel_left_joint>
    <encoder_wheel_right_joint>front_right_wheel_joint</encoder_wheel_right_joint>

    <publish_wheel_tf>false</publish_wheel_tf>
    <publish_wheel_joint_state>true</publish_wheel_joint_state>

    <actuated_wheel_diameter>0.440</actuated_wheel_diameter>
    <encoder_wheel_diameter>0.540</encoder_wheel_diameter>
    <wheel_separation>1.500</wheel_separation>

    <odometry_source>0</odometry_source>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_footprint</robot_base_frame>

    <max_wheel_acceleration>3.0</max_wheel_acceleration>
    <max_wheel_deceleration>3.0</max_wheel_deceleration>
    <max_wheel_speed_tolerance>0.05</max_wheel_speed_tolerance>
    <max_wheel_torque>20000</max_wheel_torque>
    <max_steering_speed>3.14</max_steering_speed>
    <max_steering_angle_tolerance>0.2</max_steering_angle_tolerance>

  </plugin>

  \endcode
*/

class GazeboRosAckermannDrive2 : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosAckermannDrive2();

  /// Destructor
  ~GazeboRosAckermannDrive2();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosAckermannDrive2Private> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_ACKERMANN_FROM_TRICYCLE_DRIVE_HPP_
