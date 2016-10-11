### Execute the service pythomm first

```xml
<node pkg="dynamic_reconfig" name="dynamic_server" type="dynamic_server.py" output="screen"/>
```

### Bash testing command

 - test service only

```bash
rosservice call /dynamic_update "{node_name: '/move_base/global_costmap/inflation_layer/', config_name: 'inflation_radius', config_type: 'float', new_cnofig: '40.0'}"
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

### Features

 - Change spesific defined config
 - Cnange many configs at a time

### plan to do

 - Change parameters from \*.yaml
