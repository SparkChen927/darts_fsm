//
// Created by spark on 2022/3/2.
//
#include "darts_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle &nh) {

}

bool StateMachine::isAim(rm_msgs::DbusData data_dbus_) {
  return false;
}

bool StateMachine::isReady(rm_msgs::DbusData data_dbus_) {
  return false;
}

bool StateMachine::isPush(rm_msgs::DbusData data_dbus_) {
  return false;
}

bool StateMachine::isBack(rm_msgs::DbusData data_dbus_) {
  return false;
}

void StateMachine::initAim() {

}

void StateMachine::initReady() {

}

void StateMachine::initPush() {

}

void StateMachine::initBack() {

}
