//
// Created by spark on 2022/4/27.
//
#include "darts_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle &nh, ros::NodeHandle& controller_nh) : context_(*this) {
  config_ = {
      .qd_15 = getParam(controller_nh, "qd_15", 0.),
      .qd_30 = getParam(controller_nh, "qd_30", 0.),
      .ctrl_vel = getParam(controller_nh, "ctrl_vel", 0.),
  };
  push_per_rotation_ = getParam(controller_nh, "push_per_rotation", 0);
  dbus_sub_ = nh_.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &StateMachine::dbusCB, this);
  context_.enterStartState();
}

void StateMachine::Gimbal(const ros::Time& time, const ros::Duration& period) {
  ctrl_yaw_.setCommand(config_.ctrl_vel);
  ctrl_pitch_.setCommand(config_.ctrl_vel);
  ctrl_yaw_.update(time, period);
  ctrl_pitch_.update(time, period);
}

void StateMachine::initReady() {
  ROS_INFO("Enter Ready");
  if(s_struct_.s_r_ == rm_msgs::DbusData::UP)
  {
    ctrl_friction_l_.setCommand(config_.qd_15);
    ctrl_friction_r_.setCommand(-config_.qd_15);
  }
  else if(s_struct_.s_r_ == rm_msgs::DbusData::MID)
  {
    ctrl_friction_l_.setCommand(config_.qd_30);
    ctrl_friction_r_.setCommand(-config_.qd_30);
  }
  else
  {
    ROS_ERROR("Failed to get ready");
  }
}

void StateMachine::initPush() {
  ROS_INFO("Enter Push");
    ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                             2. * M_PI / static_cast<double>(push_per_rotation_));
}

void StateMachine::initBack() {
  ROS_INFO("Enter Back");
  ctrl_trigger_.setCommand(-(ctrl_trigger_.command_struct_.position_ -
                             2. * M_PI / static_cast<double>(push_per_rotation_)));
}

void StateMachine::commandCB(const rm_msgs::DbusData::_s_r_type &msg) {
  s_struct_.s_r_ = msg;
  cmd_rt_buffer_.writeFromNonRT(s_struct_);
}

