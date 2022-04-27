//
// Created by spark on 2022/4/27.
//
#pragma once

//#include "StateMachine_sm.h"
#include <ros/ros.h>
#include <rm_msgs/DbusData.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <rm_common/ros_utilities.h>
#include <rm_msgs/ShootCmd.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/decision/command_sender.h>

class StateMachine{
public:
  StateMachine(ros::NodeHandle &nh);

  bool isAim(rm_msgs::DbusData data_dbus_) {
    return false;
  }

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

  void initAim();

  void initReady(rm_msgs::DbusData data_dbus_);

  void initPush();

  void initBack();

private:
  rm_common::ShooterCommandSender *shooter_cmd_sender_{};
};