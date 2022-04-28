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
  context_.enterStartState();
}

void StateMachine::Gimbal(rm_msgs::DbusData data_dbus_) {
  gimbal_cmd_sender_->setRate(data_dbus_.ch_l_x,data_dbus_.ch_l_y);
}

void StateMachine::initReady() {
  ROS_INFO("Enter Ready");
}

void StateMachine::Ready() {

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

