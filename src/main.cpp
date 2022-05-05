//
// Created by spark on 2022/5/5.
//

#include "darts_fsm/StateMachine.h"

int main(int argc, char **argv) {
  std::string robot;
  ros::init(argc, argv, "darts_fsm");
  ros::NodeHandle darts_nh("~");
  robot = getParam(darts_nh, "robot_type", (std::string) "error");
  return 0;
}
