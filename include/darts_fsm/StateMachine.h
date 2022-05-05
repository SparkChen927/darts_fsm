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
#include <rm_common/referee/referee.h>

struct Config
{
  double qd_15, qd_30, lf_extra_rotat_speed;
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

  void Gimbal(rm_msgs::DbusData data_dbus_);

  void initReady();
  void Ready();

  void initPush();
  void Push();

  void initBack();
  void Back();

  void dbusCB(const rm_msgs::DbusData::ConstPtr &dbus_data) {
    dbus_ = *dbus_data;
    context_.dbusUpdate(*dbus_data);
  }

  void getReady(rm_msgs::DbusData data_dbus_);

protected:
  StateMachineContext context_;

private:
  rm_common::GimbalCommandSender *gimbal_cmd_sender_{};
  rm_common::ShooterCommandSender *shooter_cmd_sender_{};
  ros::NodeHandle nh_;
  rm_common::RefereeData data_;
  rm_msgs::DbusData dbus_;
  rm_msgs::DbusData dbus_data_;
  effort_controllers::JointPositionController ctrl_trigger_;
  effort_controllers::JointVelocityController ctrl_friction_l_, ctrl_friction_r_;
  ros::Subscriber dbus_sub_;
  int ch_r_state_;
  Config config_{};
  int push_per_rotation_{};
};