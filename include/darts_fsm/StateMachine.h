//
// Created by spark on 2022/4/27.
//
#pragma once

//#include "StateMachine_sm.h"
#include <ros/ros.h>
#include <rm_msgs/DbusData.h>

class StateMachine{
public:
  StateMachine(ros::NodeHandle &nh);

  bool isAim(rm_msgs::DbusData data_dbus_);//pitch&raw

  bool isReady(rm_msgs::DbusData data_dbus_);

  bool isPush(rm_msgs::DbusData data_dbus_);

  bool isBack(rm_msgs::DbusData data_dbus_);//回调

  void initAim();

  void initReady();

  void initPush();

  void initBack();

};