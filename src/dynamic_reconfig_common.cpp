#include <sstream>
#include <dynamic_reconfig_common.h>
#include "ros/ros.h"

namespace dynamic_reconfig_common {

const std::string DynamicUpdateServiceClient::ServiceName = DynamicUpdateServiceName;

template<class Server>
BaseServiceClient<Server>::BaseServiceClient(ros::NodeHandle& nh_,
                                             const std::string& ServerName) {
  client = nh_.serviceClient<Server>(ServerName);
}

template<class Server>
BaseServiceClient<Server>::~BaseServiceClient() {
}

template<class Server>
bool BaseServiceClient<Server>::ExecuteServiceCall(int re_try) {
  if (!client.call(srv)) {
    int i = 0;
    while (!client.call(srv)) {
      if (i > re_try) {
        return false;
      }
      i++;
    }
  }
  return true;
}

DynamicUpdateServiceClient::DynamicUpdateServiceClient(ros::NodeHandle& nh_)
    : BaseServiceClient(nh_, ServiceName) {
}

bool DynamicUpdateServiceClient::ConfigUpdate(
       std::string node_name, std::string config_name,
       std::string config_type, std::string new_config) {
  srv.request.node_name = node_name;
  srv.request.config_name = config_name;
  srv.request.config_type = config_type;
  srv.request.new_config = new_config;
  return ExecuteServiceCall(3);
}

const std::string NavGlobalCostmapInflationLayerNode = "/move_base/global_costmap/inflation_layer/";
const std::string InflationLayerInflationRadius = "inflation_radius";

bool DynamicUpdateServiceClient::ChangeInflationRadius(double number) {
  bool rtn = false;
  std::stringstream ss;
  ss << number;
  std::string str_number = ss.str();
  rtn = ConfigUpdate(NavGlobalCostmapInflationLayerNode, InflationLayerInflationRadius,
    DynamicUpdateTypeFloat, str_number);
  return rtn;
}

};
