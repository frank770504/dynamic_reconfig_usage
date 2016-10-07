#include <sstream>
#include <dynamic_reconfig_common.h>
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_reconfig_common_test");
  ros::NodeHandle n;
  dynamic_reconfig_common::DynamicUpdateServiceClient DUSC(n);
  if (atoi(argv[1]) == 0) {
    // example: I will extract some common usage out as particular functions,
    // in this case, we can kust give the radius and it will update that
    bool rtn = DUSC.ChangeInflationRadius(atof(argv[2]));
    ROS_INFO_STREAM("result: " << rtn);
  } else if (atoi(argv[1]) == 1) {
    // example: this shows how to use this api to call the dynamic reconfig service
    // After giving ConfigUpdate(), node_name, config_name, type, and new_config,
    // it will update the config.
    const std::string NodeName = "/move_base/global_costmap/inflation_layer/";
    const std::string ConfigName = "inflation_radius";
    std::stringstream ss;
    ss << argv[2];
    std::string str_number = ss.str();
    bool rtn = DUSC.ConfigUpdate(NodeName, ConfigName,
      dynamic_reconfig_common::DynamicUpdateTypeFloat, str_number);
    ROS_INFO_STREAM("result: " << rtn);
  }
  return 0;
}
