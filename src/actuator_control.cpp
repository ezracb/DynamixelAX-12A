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

#include "actuator_controller/actuator_control.h"


ActuatorControl::ActuatorControl()
    :node_handle_(""),
     dxl_cnt_(2)
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyACM0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 1000000);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

  dxl_id_[0] = node_handle_.param<int>("left_wheel", 1);
  dxl_id_[1] = node_handle_.param<int>("right_wheel", 2);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);
  printf("ID : %u\n", dxl_id_[0]);
  printf("ID : %u\n", dxl_id_[1]);

    //Protocol 1.0

   uint8_t id_cnt = 2;
    dxl_wb_->scan(&dxl_id_[1], &id_cnt, 10);
    dxl_wb_->scan(&dxl_id_[0], &id_cnt, 10);
    printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d\n",
              dxl_id_[0], dxl_wb_->getModelName(1), dxl_baud_rate);
    printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d\n",
              dxl_id_[1], dxl_wb_->getModelName(2), dxl_baud_rate);


  initMsg();

  // for (int index = 0; index < dxl_cnt_; index++)
  //   dxl_wb_->wheelMode(dxl_id_[index], profile_velocity, profile_acceleration);

  dxl_wb_->addSyncWrite("Goal_Velocity");

  initPublisher();
  initServer();
  initImuSubcriber();
  initVelocitySubscriber();
    ROS_INFO("sub");
  initJointStatesPublisher();
//    ROS_INFO("JOINTsub");
  initOdomPublisher();
}

ActuatorControl::~ActuatorControl()
{
  for (int index = 0; index < 2; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void ActuatorControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("  Dynamixel Controller with OpenCM9.04 for TurtleBot 3         \n");
  printf("-----------------------------------------------------------------------\n");
  printf("\n");

  for (int index = 0; index < dxl_cnt_; index++)
  {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
    printf("ID      : %d\n", dxl_id_[index]);
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");

}

void ActuatorControl::initPublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 10);
}

void ActuatorControl::initVelocitySubscriber()
{
  geometry_msgs::Twist msg;
  command_velocity_ = node_handle_.subscribe("cmd_vel",1, &ActuatorControl::commandVelocitySubscriber,this);

}

void ActuatorControl::initImuSubcriber()
{
  imu_orientation = node_handle_.subscribe("imu",1000, &ActuatorControl::imuSubcriber, this);
}

void ActuatorControl::initOdomPublisher()
{
  odom_pub = node_handle_.advertise<nav_msgs::Odometry>("odom", 10);
}

void ActuatorControl::initJointStatesPublisher()
{
  joint_states_pub = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);
}

void ActuatorControl::initServer()
{
  wheel_command_server_ = node_handle_.advertiseService("wheel_command", &ActuatorControl::wheelCommandMsgCallback, this);
}

void ActuatorControl::imuSubcriber(const sensor_msgs::Imu::ConstPtr& msg)
{
//  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
//  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
//  ROS_INFO("Imu Angular Velocity z: [%f]", msg->angular_velocity.z+(22/7));
  orientationRobot = msg->angular_velocity.z;
}


void ActuatorControl::commandVelocitySubscriber(const geometry_msgs::Twist::ConstPtr& msg){
  float goalLinearVelocity = msg->linear.x;
  float goalAngularVelocity = msg->angular.z;
  static float oldgoalLinearVelocity = 0, oldgoalAngularVelocity = 0;
  if (oldgoalLinearVelocity != goalLinearVelocity || oldgoalAngularVelocity != goalAngularVelocity)
  {
    speedLeft = (goalLinearVelocity-(goalAngularVelocity*0.16/2))*1023/0.2;
    speedRight = (goalLinearVelocity+(goalAngularVelocity*0.16/2))*-1023/0.2;
    if (speedLeft > 1023.0)speedLeft=1023.0;
    if (speedLeft < -1023.0)speedLeft=-1023.0;
    if (speedRight > 1023.0)speedRight=1023.0;
    if (speedRight < -1023.0)speedRight=-1023.0;
    ROS_INFO("speedLeft: %f, speedRight: %f", speedLeft, speedRight);
    dxl_wb_->goalSpeed(1,speedLeft);
    dxl_wb_->goalSpeed(2,speedRight);
    oldgoalLinearVelocity = goalLinearVelocity;
    oldgoalAngularVelocity = goalAngularVelocity;
  }
}

void ActuatorControl::OdomPublisher()
{
  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
  sensor_msgs::JointState joint_states;
  static double x = 0.0;
  static double y = 0.0;
  static double th = 0.0;

  static double vx = 0.0;
  static double vy = 0.0;
  static double vth = 0.0;
  static double v_left = 0.0,v_left_past=0.0;
  static double v_right = 0.0,v_right_past=0.0;
  static bool statusL = true, statusR = true;
  static double d = 0.16, r=0.033;

  static double posL,posL_past,posR,posR_past;

  static float joint_states_pos[2] = {0.0, 0.0};
  static float joint_states_vel[2] = {0.0, 0.0};
  static float joint_states_eff[2] = {0.0, 0.0};

  // static char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint"};
   joint_states.header.frame_id = "base_link";
   joint_states.name.resize(2);
   joint_states.velocity.resize(2);
   joint_states.position.resize(2);
   joint_states.name[0]            = "wheel_left_joint";
   joint_states.name[1]            = "wheel_right_joint";
  //joint_states.name            = joint_states_name;
  //  joint_states.name_length     = 2;
  //  joint_states.position_length = 2;
  //  joint_states.velocity_length = 2;
  //  joint_states.effort_length   = 2;


  ActuatorControl::current_time = ros::Time::now();

  v_left  = dxl_wb_->itemRead(dxl_id_[0], "Present_Velocity");
  v_right = dxl_wb_->itemRead(dxl_id_[1], "Present_Velocity");
  posL = dxl_wb_->itemRead(dxl_id_[0], "Present_Position");
  posR = dxl_wb_->itemRead(dxl_id_[1], "Present_Position");

  //Velocity 0 when no trigger
  // if(speedLeft==0)
  // {
  //   v_left=0;
  //   v_left_past=0;
  //   ROS_INFO("STOPLEFT");
  // }
  // else
  // {
  //   v_left  = dxl_wb_->itemRead(dxl_id_[0], "Present_Velocity");
  // }
  // if(speedRight==0)
  // {
  //   v_right=0;
  //   v_right_past=0;
  //   ROS_INFO("STOPRIGHT");
  // }
  // else
  // {
  //   v_right = dxl_wb_->itemRead(dxl_id_[1], "Present_Velocity");
  // }

  // Value to Speed RPM
  v_left = v_left/10;
  if (v_left >102.3)v_left=v_left-102.3;
  else v_left=-v_left;
  v_right = v_right/10;
  if (v_right >102.3)v_right=-(v_right-102.3);
  else v_right=v_right;


  //Compensate the error of AX-12A

  //Left Motor Filter
  if (speedLeft>0) //Backward
  {//Position Increment
    // ROS_INFO("MUNDUR");
    if (posL==1023 && statusL==true)
    {
      statusL=false;
      v_left = v_left_past;
      // ROS_INFO("BLIND vLeft: %f,posLeft: %f", v_left,posL);
    }
    else if (statusL==false)
    {
      v_left = v_left_past;
      // ROS_INFO("REMOVE vLeft: %f,posLeft: %f", v_left,posL);
      if (posL_past==0 && posL>0)
      {
        if (v_left<-65 || v_left>0)v_left=v_left_past;
        else v_left_past = v_left;
        joint_states_pos[0]  -= (0.29*M_PI/180)*((360/0.29)-1023);
        joint_states_pos[0]  += 0.29*M_PI/180*(posL_past-posL);
        statusL=true;
        // ROS_INFO("GOOD vLeft: %f,posLeft: %f", v_left,posL);
      }
    }
    else
    {
      if (v_left<-65 || v_left>0)v_left=v_left_past;
      else v_left_past = v_left;
      joint_states_pos[0]  += 0.29*M_PI/180*(posL_past-posL);
      // ROS_INFO("SAVE vLeft: %f,posLeft: %f", v_left,posL);
    }
  }
  else if (speedLeft<0)//Forward
  {//Position Decrement
    // ROS_INFO("MAJU");
    if (posL==0 && statusL==true)
    {
      statusL=false;
      v_left = v_left_past;
      // ROS_INFO("BLIND vLeft: %f,posLeft: %f", v_left,posL);
    }
    else if (statusL==false)
    {
      v_left = v_left_past;
      // ROS_INFO("REMOVE vLeft: %f,posLeft: %f", v_left,posL);
      if (posL_past==1023 && posL<1023)
      {
        if (v_left>65 || v_left<0)v_left=v_left_past;
        else v_left_past = v_left;
        joint_states_pos[0]  += (0.29*M_PI/180)*((360/0.29)-1023);
        joint_states_pos[0]  += 0.29*M_PI/180*(posL_past-posL);
        statusL=true;
        // ROS_INFO("GOOD vLeft: %f,posLeft: %f", v_left,posL);
      }
    }
    else
    {
      if (v_left>65 || v_left<0)v_left=v_left_past;
      else v_left_past = v_left;
      joint_states_pos[0]  += 0.29*M_PI/180*(posL_past-posL);
      // ROS_INFO("SAVE vLeft: %f,posLeft: %f", v_left,posL);
    }
  }
  else if (speedLeft==0)
  {
    v_left = 0;
  }
  joint_states_vel[0]  = v_left;
  posL_past=posL;

  //Right Motor Filter
  if (speedRight>0) //Forward
  {//Position Increment
    // ROS_INFO("MAJU");

    if (posR==1023 && statusR==true)
    {
      statusR=false;
      v_right = v_right_past;
      // ROS_INFO("BLIND vright: %f,posright: %f", v_right,posR);
    }
    else if (statusR==false)
    {
      v_right = v_right_past;
      // ROS_INFO("REMOVE vright: %f,posright: %f", v_right,posR);
      if (posR_past==0 && posR>0)
      {
        if (v_right>65 || v_right<0)v_right=v_right_past;
        else v_right_past = v_right;
        joint_states_pos[1]  += (0.29*M_PI/180)*((360/0.29)-1023);
        joint_states_pos[1]  += 0.29*M_PI/180*(posR-posR_past);
        statusR=true;
        // ROS_INFO("GOOD vright: %f,posright: %f", v_right,posR);
      }
    }
    else
    {
      if (v_right>65 || v_right<0)v_right=v_right_past;
      else v_right_past = v_right;
      joint_states_pos[1]  += 0.29*M_PI/180*(posR-posR_past);
      // ROS_INFO("SAVE vright: %f,posright: %f", v_right,posR);
    }
  }
  else if (speedRight<0)//Backward
  {//Position Decrement
    // ROS_INFO("MUNDUR");
    if (posR==0 && statusR==true)
    {
      statusR=false;
      v_right = v_right_past;
      // ROS_INFO("BLIND vright: %f,posright: %f", v_right,posR);
    }
    else if (statusR==false)
    {
      v_right = v_right_past;
      // ROS_INFO("REMOVE vright: %f,posright: %f", v_right,posR);
      if (posR_past==1023 && posR<1023)
      {
        if (v_right<-65 || v_right>0)v_right=v_right_past;
        else v_right_past = v_right;
        joint_states_pos[1]  -= (0.29*M_PI/180)*((360/0.29)-1023);
        joint_states_pos[1]  += 0.29*M_PI/180*(posR-posR_past);
        statusR=true;
        // ROS_INFO("GOOD vright: %f,posright: %f", v_right,posR);
      }
    }
    else
    {
      if (v_right<-65 || v_right>0)v_right=v_right_past;
      else v_right_past = v_right;
      joint_states_pos[1]  += 0.29*M_PI/180*(posR-posR_past);
      // ROS_INFO("SAVE vright: %f,posright: %f", v_right,posR);
    }
  }
  else if (speedRight==0)
  {
    v_right = 0;
  }
  joint_states_vel[1]  = v_right;
  posR_past=posR;

  //v_right & v_left unit RPM, Odometry unit is m/s
  //convert from RPM to m/s
  v_right =  v_right * 2 * M_PI * r / 60;
  v_left =  v_left * 2 * M_PI * r / 60;


  // Omega Orientation from IMU Sensor
  th=-orientationRobot;

  //ROS_INFO("vLeft: %f,posLeft: %f,vRight: %f,posRight: %f", v_left,posL,v_right,posR);
  vx = -(v_left + v_right)/2 ;
  vy = -vx * sin(th);

  //  Orientation from Different speed of Motor
    //  vth = ( v_right - v_left ) / d;
    //  double delta_th = vth * dt;
    //   th += delta_th;

  //compute odometry in a typical way given the velocities of the robot
   double dt = (current_time - last_time).toSec();
   double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
   double delta_y = (vx * sin(th) + vy * cos(th)) * dt;


   x += delta_x; //pose x
   y += delta_y; //pose y


   //since all odometry is 6DOF we'll need a quaternion created from yaw
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

   //first, we'll publish the transform over tf
   geometry_msgs::TransformStamped odom_trans;
   odom_trans.header.stamp = current_time;
   odom_trans.header.frame_id = "odom";
   odom_trans.child_frame_id = "base_footprint";

   odom_trans.transform.translation.x = x;
   odom_trans.transform.translation.y = y;
   odom_trans.transform.translation.z = 0.0;
   odom_trans.transform.rotation = odom_quat;

   //send the transform
   ActuatorControl::odom_broadcaster.sendTransform(odom_trans);

   //next, we'll publish the odometry message over ROS
   nav_msgs::Odometry odom;
   odom_trans.header = odom.header;
   odom.header.stamp = current_time;
   odom.header.frame_id = "odom";

   //set the position
   odom.pose.pose.position.x = x;
   odom.pose.pose.position.y = y;
   odom.pose.pose.position.z = 0.0;
   odom.pose.pose.orientation = odom_quat;

   //set the velocity
   odom.child_frame_id = "base_link";
   odom.twist.twist.linear.x = vx;
   odom.twist.twist.linear.y = vy;
   odom.twist.twist.angular.z = vth;

   //publish the message
   odom_pub.publish(odom);

   joint_states.position[0] = joint_states_pos[0];
   joint_states.position[1] = joint_states_pos[1];
   joint_states.velocity[0] = joint_states_vel[0];
   joint_states.velocity[1] = joint_states_vel[1];
//   joint_states.velocity[0] = 10;
   joint_states.header.stamp = current_time;
  joint_states_pub.publish(joint_states);

   ActuatorControl::last_time = current_time;
}

void ActuatorControl::dynamixelStatePublish()
{
  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

  for (int index = 0; index < dxl_cnt_; index++)
  {
    dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
    dynamixel_state[index].id                  = dxl_id_[index];
    dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
    dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
    dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
    dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
    dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
    dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
  }
  dynamixel_state_list_pub_.publish(dynamixel_state_list);
}

void ActuatorControl::controlLoop()
{
  OdomPublisher();
  dynamixelStatePublish();
}

bool ActuatorControl::wheelCommandMsgCallback(dynamixel_workbench_msgs::WheelCommand::Request &req,
                                              dynamixel_workbench_msgs::WheelCommand::Response &res)
{
  static int32_t goal_velocity[2] = {0, 0};

  goal_velocity[0] = dxl_wb_->convertVelocity2Value(dxl_id_[0], req.left_vel);
  goal_velocity[1] = dxl_wb_->convertVelocity2Value(dxl_id_[1], (-1) * req.right_vel);

  bool ret = dxl_wb_->syncWrite("Goal_Velocity", goal_velocity);

  res.result = ret;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "actuator_control");
  ActuatorControl vel_ctrl;

  ros::Rate loop_rate(250);
  while (ros::ok())
  {
    vel_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
