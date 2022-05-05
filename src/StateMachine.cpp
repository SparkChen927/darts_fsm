//
// Created by spark on 2022/3/2.
//
#include "darts_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle &nh) : context_(*this), controller_manager_(nh) {
  nh_ = ros::NodeHandle(nh);
  ros::NodeHandle gimbal_nh(nh_, "gimbal");
  ros::NodeHandle shooter_nh(nh_, "shooter");
  gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh, data_);
  shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, data_);
  dbus_sub_ = nh_.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &StateMachine::dbusCB, this);
  context_.enterStartState();
}

void StateMachine::Gimbal(rm_msgs::DbusData data_dbus_) {
  gimbal_cmd_sender_->setRate(data_dbus_.ch_l_x,data_dbus_.ch_l_y);
}

void StateMachine::initReady() {
  ROS_INFO("Enter Ready");
}

void StateMachine::Ready() {
  getReady(dbus_data_);
  if(ch_r_state_ == 1)
  {
    ctrl_friction_l_.setCommand(unknown_num_);
    ctrl_friction_r_.setCommand(unknown_num_);
  }
  else if(ch_r_state_ == 2)
  {
    ctrl_friction_l_.setCommand(unknown_num_);
    ctrl_friction_r_.setCommand(unknown_num_);
  }
  else
  {
    ROS_ERROR("Failed to get ready");
  }
}

void StateMachine::initPush() {
  ROS_INFO("Enter Push");
}

void StateMachine::Push() {
  if(ch_r_state_ == 1)
  {
    ctrl_trigger_.setCommand(unknown_num_);
  }
  else if(ch_r_state_ == 2)
  {
    ctrl_trigger_.setCommand(unknown_num_);
  }
}

void StateMachine::initBack() {
  ROS_INFO("Enter Back");
}

void StateMachine::Back() {
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void StateMachine::getReady(rm_msgs::DbusData data_dbus_) {
  if(data_dbus_.s_r == rm_msgs::DbusData::UP)
  {
    ch_r_state_ = 1;
  }
  else if(data_dbus_.s_r == rm_msgs::DbusData::MID)
  {
    ch_r_state_ = 2;
  }
}

