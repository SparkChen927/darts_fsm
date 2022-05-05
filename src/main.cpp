//
// Created by spark on 2022/5/5.
//

#include "darts_fsm/StateMachine.h"

int main(int argc, char **argv) {
  std::string robot;
  ros::init(argc, argv, "darts_fsm");
  ros::NodeHandle darts_nh("~");
  robot = getParam(darts_nh, "robot_type", (std::string) "error");
  StateMachine sm(darts_nh);
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::Time time = ros::Time::now();
    ros::Time begin_time = ros::Time::now();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
