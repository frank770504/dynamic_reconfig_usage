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


const std::string DynamicUpdateServiceClient::kScenarioListKey_ =
  "dynamic_update_define/scenario_list";

DynamicUpdateServiceClient::DynamicUpdateServiceClient(ros::NodeHandle& nh_)
    : BaseServiceClient(nh_, ServiceName), nh_(nh_) {
  scenario_list_.resize(0);
  if (!nh_.getParam(kScenarioListKey_, yml_params_)) {
    ROS_ERROR_STREAM("get" << kScenarioListKey_ << "error");
    return;
  } else {
    // load scenario name
    for (int i = 0; i < yml_params_.size(); i++) {
      scenario_list_.push_back(yml_params_[i]);
    }
  }
}

void DynamicUpdateServiceClient::ClearUpdateArray() {
  srv.request.node_name.clear();
  srv.request.config_name.clear();
  srv.request.config_type.clear();
  srv.request.new_config.clear();
  srv.request.node_name.resize(0);
  srv.request.config_name.resize(0);
  srv.request.config_type.resize(0);
  srv.request.new_config.resize(0);
}

void DynamicUpdateServiceClient::AddToUpdateArray(
       std::string node_name, std::string config_name,
       std::string config_type, std::string new_config) {
  srv.request.node_name.push_back(node_name);
  srv.request.config_name.push_back(config_name);
  srv.request.config_type.push_back(config_type);
  srv.request.new_config.push_back(new_config);
}

bool DynamicUpdateServiceClient::ConfigUpdate(
       std::string node_name, std::string config_name,
       std::string config_type, std::string new_config) {
  // update one config only
  ClearUpdateArray();
  AddToUpdateArray(node_name, config_name, config_type, new_config);
  return ExecuteServiceCall(3);
}

bool DynamicUpdateServiceClient::ConfigUpdate() {
  if (srv.request.node_name.size() <= 0) {
    ROS_ERROR_STREAM("Config array is empty. Do you use AddToUpdateArray() before?");
    return false;
  }
  return ExecuteServiceCall(3);
}

const std::string NavGlobalCostmapInflationLayerNode = "/move_base/global_costmap/inflation_layer/";
const std::string InflationLayerInflationRadius = "inflation_radius";

bool DynamicUpdateServiceClient::ChangeInflationRadius(double number) {
  bool rtn = false;
  std::stringstream ss;
  ss << number;
  std::string str_number = ss.str();
  ClearUpdateArray();
  rtn = ConfigUpdate(NavGlobalCostmapInflationLayerNode, InflationLayerInflationRadius,
    DynamicUpdateTypeFloat, str_number);
  return rtn;
}

void DynamicUpdateServiceClient::YamlParseTest(std::vector<std::string> enabled_list) {
  for (ScenarioIter it = scenario_list_.begin(); it != scenario_list_.end(); ++it) {
    for (ScenarioIter iv = enabled_list.begin(); iv != enabled_list.end(); ++iv) {
      if (*it == *iv) {
        std::string scenario = *iv;
        std::string node_name;
        ROS_INFO_STREAM("starting change config from " << scenario);
        if (!nh_.getParam(scenario + "/node_name", node_name)) {
          ROS_ERROR_STREAM("get " << scenario + "/node_name" << " error");
          continue;
        }
        ROS_INFO_STREAM("node name is " << node_name);
      }
    }
  }
  ROS_INFO_STREAM("");
}

};
