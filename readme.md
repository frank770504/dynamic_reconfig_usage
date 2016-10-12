### Execute the service pythomm first

```xml
<node pkg="dynamic_reconfig" name="dynamic_server" type="dynamic_server.py" output="screen"/>
```

### Features

 - Change spesific defined config
 - Cnange many configs at a time
 - Change parameters from \*.yaml

### Bash testing command

 - test service only

```bash
rosservice call /DynamicUpdate  "{node_name: ['/move_base/global_costmap/inflation_layer/'], config_name: ['inflation_radius'], config_type: ['float'], new_config: ['40.0']}"
```

 - using self-test node
   - the node is used for show how to use the client api
   - It can only change the inflation radius temporarily
   - The below command would change the radius to 30.0

```bash
rosrun dynamic_reconfig dynamic_reconfig_client_test_node 0 30.0
```

### API usage

 - See dynamic_reconfig_client_test.cpp

---

## Using Yaml to Do Config.

### Write A YAML Example


```
dynamic_update_define:
  scenario_list:
   - local_planner_scenario_1
   - local_planner_scenario_2
   - inflation_scenario_1
   - inflation_scenario_2

local_planner_scenario_1:
  node_name: "/move_base/DWAPlannerROS/"
  float_configs:
    max_vel_x: 0.60
    min_vel_x: 0.07
    max_vel_y: 0.4
    min_vel_y: -0.4

local_planner_scenario_2:
  node_name: "/move_base/DWAPlannerROS/"
  float_configs:
    max_vel_x: 0.45
    min_vel_x: 0.01
    max_vel_y: 0.2
    min_vel_y: -0.2
    
inflation_scenario_1:
  node_name: "/move_base/global_costmap/inflation_layer/"
  bool_configs:
    enabled: 1
  float_configs:
    inflation_radius: 12.5

inflation_scenario_2:
  node_name: "/move_base/global_costmap/inflation_layer/"
  bool_configs:
    enabled: 0
  float_configs:
    inflation_radius: 40.0
```

### Yaml Details Explaination

```
dynamic_update_define:
  scenario_list:
   - local_planner_scenario_1
   - local_planner_scenario_2
   - inflation_scenario_1
   - inflation_scenario_2

```

1. I will find wll the scenario names under "dynamic_update_define/scenario_list"
1. Each Scenario names shows the group of parameters under the same node name

```
local_planner_scenario_1:
  node_name: "/move_base/DWAPlannerROS/"
  float_configs:
    max_vel_x: 0.60
    min_vel_x: 0.07
    max_vel_y: 0.4
    min_vel_y: -0.4

inflation_scenario_2:
  node_name: "/move_base/global_costmap/inflation_layer/"
  bool_configs:
    enabled: 0
  float_configs:
    inflation_radius: 40.0
```

1. For each scenario, it has two kind of parameters. One is the node_name, and the other is $(Type)_configs
1. The node_name will guide us to the right dynamic server.
1. There are 4 types in $(Type)_configs, which are "float_configs", "bool_configs", "int_configs" and "string_configs"
1. The usage is easy. All we have to do is add the config name and value under the right type tag.
1. e,g, I put "max_vel_x: 0.60" under "float_configs:", and "enabled: 0" under "bool_configs:"

### Rosparameter

After setting the yaml, we have to add the parameters into rosparam server

There are two ways:
 - rosparam load dynamic_update_config_alter.yaml
 - <rosparam file="dynamic_update_config_alter.yaml" command="load"/>

### API usage

It can change parameter by using the scenario names.

See dynamic_reconfig_client_test.cpp for the detail.

Note: at "} else if (atoi(argv[1]) == 4) {"
