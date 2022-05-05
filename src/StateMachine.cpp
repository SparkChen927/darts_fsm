//
// Created by spark on 2022/4/27.
//
#include "darts_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle &nh, ros::NodeHandle& controller_nh) : context_(*this) {
  config_ = {
             .qd_15 = getParam(controller_nh, "qd_15", 0.),
             .qd_30 = getParam(controller_nh, "qd_30", 0.),
             .lf_extra_rotat_speed = getParam(controller_nh, "lf_extra_rotat_speed", 0.)
  };
  push_per_rotation_ = getParam(controller_nh, "push_per_rotation", 0);
  nh_ = ros::NodeHandle(nh);
  ros::NodeHandle gimbal_nh(nh_, "gimbal");
  gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh, data_);
  ros::NodeHandle shooter_nh(nh_, "shooter");
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
    ctrl_friction_l_.setCommand(config_.qd_15 + config_.lf_extra_rotat_speed);
    ctrl_friction_r_.setCommand(-config_.qd_15);
  }
  else if(ch_r_state_ == 2)
  {
    ctrl_friction_l_.setCommand(config_.qd_30 + config_.lf_extra_rotat_speed);
    ctrl_friction_r_.setCommand(-config_.qd_30);
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
    ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                             2. * M_PI / static_cast<double>(push_per_rotation_));
  }
  else if(ch_r_state_ == 2)
  {
    ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                             2. * M_PI / static_cast<double>(push_per_rotation_));
  }
}

void StateMachine::initBack() {
  ROS_INFO("Enter Back");
}

void StateMachine::Back() {
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  ctrl_trigger_.setCommand(-(ctrl_trigger_.command_struct_.position_ -
                           2. * M_PI / static_cast<double>(push_per_rotation_)));
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

