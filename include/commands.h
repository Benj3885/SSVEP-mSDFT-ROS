#ifndef EEG_COMMANDS_H
#define EEG_COMMANDS_H

#include "ros/ros.h"
#include "ros/advertise_options.h"
#include "std_msgs/UInt8.h"

class Command {
public:
  Command();
  void publish (uint8_t data);

private:
  ros::NodeHandle _n;
  ros::Publisher _pub;
  std_msgs::UInt8 _msg;
  ros::AdvertiseOptions _ops;
  static void connectCallback(const ros::SingleSubscriberPublisher& pub);
};

#endif
