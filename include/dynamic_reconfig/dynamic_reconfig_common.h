#ifndef DYNAMIC_RECONFIG_INCLUDE_DYNAMIC_RECONFIG_DYNAMIC_RECONFIG_COMMON_H_
#define DYNAMIC_RECONFIG_INCLUDE_DYNAMIC_RECONFIG_DYNAMIC_RECONFIG_COMMON_H_

#include <string>
#include "ros/ros.h"
#include "dynamic_reconfig/DynamicUpdate.h"

namespace dynamic_reconfig_common {

const std::string DynamicUpdateServiceName = "DynamicUpdate";

const std::string DynamicUpdateTypeInt = "int";
const std::string DynamicUpdateTypeDouble = "double";
const std::string DynamicUpdateTypeFloat = "float";
const std::string DynamicUpdateTypeString = "string";
const std::string DynamicUpdateTypeBool = "bool";

template <class Server>
class BaseServiceClient {
 public:
  BaseServiceClient(ros::NodeHandle& nh_,
                    const std::string& ServerName);
  virtual ~BaseServiceClient();
 protected:
  ros::ServiceClient client;
  Server srv;
  bool ExecuteServiceCall(int re_try);
};

class DynamicUpdateServiceClient
  : public BaseServiceClient<dynamic_reconfig::DynamicUpdate> {
  typedef std::vector<std::string>::iterator ScenarioIter;
  typedef XmlRpc::XmlRpcValue::iterator XmlIter;
 public:
  explicit DynamicUpdateServiceClient(ros::NodeHandle& nh_);
  void AddToUpdateArray(std::string node_name, std::string config_name,
         std::string config_type, std::string new_config);
  void AddParamConfigs(std::string node_name, std::string scenario, std::string config_type);
  void AddScenarioList(std::vector<std::string> enabled_list);
  void UpdateScenarioList();
  void ClearUpdateArray();
  bool ChangeInflationRadius(double number);
  bool ConfigUpdate(std::string node_name, std::string config_name,
         std::string config_type, std::string new_config);
  bool ConfigUpdate();
 protected:
 private:
  ros::NodeHandle nh_;
  static const std::string ServiceName;
  static const std::string kScenarioListKey_;
  std::vector<std::string> scenario_list_;
  XmlRpc::XmlRpcValue yml_params_;
};
}  // namespace dynamic_reconfig_common

#endif  // DYNAMIC_RECONFIG_INCLUDE_DYNAMIC_RECONFIG_DYNAMIC_RECONFIG_COMMON_H_
