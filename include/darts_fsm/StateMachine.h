//
// Created by spark on 2022/4/27.
//

#pragma once

#include "StateMachine_sm.h"
#include <ros/ros.h>
#include <rm_msgs/DbusData.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <rm_common/decision/command_sender.h>

struct Config
{
  double qd_15, qd_30, pos_raw_1, pos_pitch_1, pos_raw_2, pos_pitch_2;
};

struct s
{
  rm_msgs::DbusData data;
};

class StateMachine{
public:
  StateMachine(ros::NodeHandle &nh, ros::NodeHandle& controller_nh);

  bool isReady(rm_msgs::DbusData data_dbus_) {
    if (data_dbus_.s_l == rm_msgs::DbusData::MID)
      return true;
    else
      return false;
  }

  bool isPush(rm_msgs::DbusData data_dbus_) {
    if (data_dbus_.s_l == rm_msgs::DbusData::UP)
      return true;
    else
      return false;
  }

  bool isBack(rm_msgs::DbusData data_dbus_) {
    if (data_dbus_.s_l == rm_msgs::DbusData::DOWN)
      return true;
    else
      return false;
  }

  void Gimbal(const ros::Time& time, const ros::Duration& period);

  void initReady();

  void initPush();

  void initBack();

  void dbusCB(const rm_msgs::DbusData::ConstPtr &dbus_data) {
    dbus_ = *dbus_data;
    context_.dbusUpdate(*dbus_data);
  }

  void commandCB(const rm_msgs::DbusData::ConstPtr &msg);

protected:
  StateMachineContext context_;

private:
  ros::NodeHandle nh_;
  rm_msgs::DbusData dbus_;
  rm_msgs::DbusData dbus_data_;
  effort_controllers::JointPositionController ctrl_trigger_;
  effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;
  effort_controllers::JointVelocityController ctrl_friction_l_, ctrl_friction_r_;
  realtime_tools::RealtimeBuffer<s> cmd_rt_buffer_;
  ros::Subscriber dbus_sub_;
  s s_struct_;
  Config config_{};
  int push_per_rotation_{};
};