#!/usr/bin/env python
import roslib
import rospy

import dynamic_reconfigure.client
from dynamic_reconfig.srv import *

dynamic_update_service_name = 'DynamicUpdate'

class type_definition:
  TYPE_INT = "int"
  TYPE_FLOAT = "float"
  TYPE_DOUBLE = "double"
  TYPE_STRING = "string"
  TYPE_BOOL = "bool"

class type_converter(type_definition):
    def __init__(self, _type):
        self.set_type_(_type)
    def set_type_(self, _type):
        self.type_name_ = _type
        if self.type_name_ == self.TYPE_INT:
            self.callback_ = self.convert_to_int_
            return
        if self.type_name_ == self.TYPE_FLOAT:
            self.callback_ = self.convert_to_float_
            return
        if self.type_name_ == self.TYPE_DOUBLE:
            self.callback_ = self.convert_to_double_
            return
        if self.type_name_ == self.TYPE_STRING:
            self.callback_ = self.convert_to_string_
            return
        if self.type_name_ == self.TYPE_BOOL:
            self.callback_ = self.convert_to_bool_
            return
    def convert_to_int_(self, _string):
        return int(_string)
    def convert_to_float_(self, _string):
        return float(_string)
    def convert_to_double_(self, _string):
        return float(_string)
    def convert_to_string_(self, _string):
        return _string
    def convert_to_bool_(self, _string):
        if _string == "True" or _string == "true":
            return True
        elif _string == "False" or _string == "false":
            return False
        else:
            return bool(int(_string))
    def convert_(self, _string):
        return self.callback_(_string)

class dynamic_update_server:
    def __init__(self):
        s = rospy.Service(dynamic_update_service_name, DynamicUpdate, self.dynamic_update)
        self.req_config_name_ = "Null"
        self.req_node_name_ = "Null"
        print "Ready to do dynamic_update."
    def callback(self, config):
        for key, value in config.iteritems():
            if key == self.req_config_name_:
                rospy.loginfo( "{}{} : {}".format(self.req_node_name_, key, value) )
        #~ rospy.loginfo("Config set to {inflation_radius}, {cost_scaling_factor}, {enabled}".format(**config))
    def dynamic_update(self, req):
        print "node_name:{}, config_name:{}, new_config:{}".format(req.node_name, req.config_name, req.new_config)
        client = dynamic_reconfigure.client.Client(req.node_name, timeout=30, config_callback=self.callback)
        self.req_node_name_ = req.node_name
        self.req_config_name_ = req.config_name
        tc = type_converter(req.config_type)
        client.update_configuration({ req.config_name : tc.convert_(req.new_config)})
        return "ok"

def server_run():
    rospy.init_node('dynamic_reconfig')
    dus = dynamic_update_server()
    rospy.spin()

if __name__ == "__main__":
    try:
        server_run()
    except rospy.ROSInterruptException:
        print "ros exception"
        pass
