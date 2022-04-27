//
// Created by spark on 2022/3/2.
//
#include "darts_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle &nh) {

}

void StateMachine::initAim() {
  ROS_INFO("Enter Aim");
}

void StateMachine::initReady(rm_msgs::DbusData data_dbus_) {
  ROS_INFO("Enter Ready");
  if (data_dbus_.s_l == rm_msgs::DbusData::UP)
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::SPEED_30M_PER_SECOND);
  }
}

void StateMachine::initPush() {
  ROS_INFO("Enter Push");
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
}

void StateMachine::initBack() {
  ROS_INFO("Enter Back");
}


