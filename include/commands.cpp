#include "commands.h"

Command::Command() {
  _ops.init<std_msgs::UInt8>("/jaco/commander", 1, &connectCallback);
  _ops.latch = true;
  _pub = _n.advertise(_ops);
}

void Command::connectCallback(const ros::SingleSubscriberPublisher& pub) {
  ROS_INFO("%s is subscribering to %s", pub.getSubscriberName().c_str(), "/jaco/commander");
}

void Command::publish (uint8_t data) {
  _msg.data = data;
  _pub.publish(_msg);
}
