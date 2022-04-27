//
// Created by spark on 2022/3/2.
//
#include "darts_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle &nh) : context_(*this), controller_manager_(nh) {
  nh_ = ros::NodeHandle(nh);
  ros::NodeHandle shooter_nh(nh_, "shooter");
  shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, data_);
  context_.enterStartState();
}

void StateMachine::initAim() {
  ROS_INFO("Enter Aim");
}

void StateMachine::Aim() {

}

void StateMachine::initReady() {
  ROS_INFO("Enter Ready");
}

void StateMachine::Ready() {

}

void StateMachine::getReady(rm_msgs::DbusData data_dbus_) {
  if (data_dbus_.s_r == rm_msgs::DbusData::UP)
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::SPEED_30M_PER_SECOND);
  }
  if (data_dbus_.s_r == rm_msgs::DbusData::MID)
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::SPEED_18M_PER_SECOND);
  }
  if (data_dbus_.s_r == rm_msgs::DbusData::DOWN)
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::SPEED_10M_PER_SECOND);
  }
}

void StateMachine::initPush() {
  ROS_INFO("Enter Push");
}

void StateMachine::Push() {
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
}

void StateMachine::initBack() {
  ROS_INFO("Enter Back");
}

void StateMachine::Back() {

}


