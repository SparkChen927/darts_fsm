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
#include <rm_common/decision/controller_manager.h>
#include <rm_common/referee/referee.h>

class StateMachine{
public:
  StateMachine(ros::NodeHandle &nh);

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

protected:
  StateMachineContext context_;

private:
  rm_common::GimbalCommandSender *gimbal_cmd_sender_{};
  rm_common::ShooterCommandSender *shooter_cmd_sender_{};
  rm_common::ControllerManager controller_manager_;
  ros::NodeHandle nh_;
  rm_common::RefereeData data_;
  rm_msgs::DbusData dbus_;
  effort_controllers::JointPositionController ctrl_trigger_;
};