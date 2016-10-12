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

  } else if (atoi(argv[1]) == 2) {
    // example: this shows how to use this api to call the dynamic reconfig service
    // When the type is bool, it is okay to send "True", "true", "False", "false",
    // or "0", "1".
    const std::string NodeName = "/move_base/global_costmap/inflation_layer/";
    const std::string ConfigName = "enabled";
    std::stringstream ss;
    ss << argv[2];
    std::string str_number = ss.str();
    bool rtn = DUSC.ConfigUpdate(NodeName, ConfigName,
      dynamic_reconfig_common::DynamicUpdateTypeBool, str_number);
    ROS_INFO_STREAM("result: " << rtn);

  } else if (atoi(argv[1]) == 3) {
    // example: this shows how to change multi-configs at a time
    // The principle of this api is to add all the needed-to-change configs into
    // arrays and do ConfigUpdate() once.
    // 1. Call ClearUpdateArray() at first to clean the array
    // 2. AddToUpdateArray() add configs we want to change
    // 3. ConfigUpdate() it will update all the configs
    DUSC.ClearUpdateArray();
    std::stringstream s1;
    s1 << argv[2];
    const std::string NodeName_DWA = "/move_base/DWAPlannerROS/";
    const std::string NodeName_Inflat = "/move_base/global_costmap/inflation_layer/";

    // NodeName_DWA
    DUSC.AddToUpdateArray(NodeName_DWA, "max_vel_x",
      dynamic_reconfig_common::DynamicUpdateTypeFloat, static_cast<std::string>(s1.str()));
    s1.str(std::string());
    s1 << argv[3];
    DUSC.AddToUpdateArray(NodeName_DWA, "min_vel_x",
      dynamic_reconfig_common::DynamicUpdateTypeFloat, static_cast<std::string>(s1.str()));

    // NodeName_Inflat
    s1.str(std::string());
    s1 << argv[4];
    DUSC.AddToUpdateArray(NodeName_Inflat, "enabled",
      dynamic_reconfig_common::DynamicUpdateTypeBool, static_cast<std::string>(s1.str()));
    s1.str(std::string());
    s1 << argv[5];
    DUSC.AddToUpdateArray(NodeName_Inflat, "inflation_radius",
      dynamic_reconfig_common::DynamicUpdateTypeFloat, static_cast<std::string>(s1.str()));

    // NodeName_DWA
    s1.str(std::string());
    s1 << argv[6];
    DUSC.AddToUpdateArray(NodeName_DWA, "max_vel_y",
      dynamic_reconfig_common::DynamicUpdateTypeFloat, static_cast<std::string>(s1.str()));
    s1.str(std::string());
    s1 << argv[7];
    DUSC.AddToUpdateArray(NodeName_DWA, "min_vel_y",
      dynamic_reconfig_common::DynamicUpdateTypeFloat, static_cast<std::string>(s1.str()));
    bool rtn = DUSC.ConfigUpdate();
    ROS_INFO_STREAM("result: " << rtn);

  } else if (atoi(argv[1]) == 4) {

    DUSC.YamlParseTest();

  }
  return 0;
}
