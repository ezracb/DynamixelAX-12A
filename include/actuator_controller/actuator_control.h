/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

/*
 *
 *     |-----------|
 *     |-----------|
 *     |-----------|
 *     |-----------|
 *     |-----------|
 *    O-------------O
 *  left          right
 *   1              2
 *
 *
 * */

#ifndef DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
#define DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H

#include <ros/ros.h>
#include "message_header.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/WheelCommand.h>

class ActuatorControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters
  //ros::Time current_time, last_time;
  ros::Time current_time = ros::Time::now();
  ros::Time last_time = ros::Time::now();
  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher joint_states_pub;


  // ROS Topic Subscriber
  ros::Subscriber command_velocity_;
  ros::Subscriber imu_orientation;

  // ROS Service Server
  ros::ServiceServer wheel_command_server_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  uint8_t dxl_id_[2];
  uint8_t dxl_cnt_;
  double orientationRobot;
  float speedLeft=0,speedRight=0;

 public:
  ActuatorControl();
  ~ActuatorControl();
  void controlLoop(void);

 private:
  void initMsg();

  void initPublisher();
  void initVelocitySubscriber();
  void initImuSubcriber();
  void initOdomPublisher();
  void initJointStatesPublisher();
  void velocitySubscriber();
  void commandVelocitySubscriber(const geometry_msgs::Twist::ConstPtr& msg);
  void imuSubcriber(const sensor_msgs::Imu::ConstPtr& msg);
  void dynamixelStatePublish();
  void OdomPublisher();
  void initServer();
  bool wheelCommandMsgCallback(dynamixel_workbench_msgs::WheelCommand::Request &req,
                               dynamixel_workbench_msgs::WheelCommand::Response &res);
};

#endif //DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
