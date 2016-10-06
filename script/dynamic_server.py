#!/usr/bin/env python
import roslib
import rospy

import dynamic_reconfigure.client
from dynamic_reconfig.srv import *

_name = "/move_base/global_costmap/inflation_layer"

class type_definition:
  TYPE_INT = "int"
  TYPE_FLOAT = "float"
  TYPE_DOUBLE = "double"
  TYPE_STRING = "string"

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
    def convert_to_int_(self, _string):
        return int(_string)
    def convert_to_float_(self, _string):
        return float(_string)
    def convert_to_double_(self, _string):
        return float(_string)
    def convert_to_string_(self, _string):
        return _string
    def convert_(self, _string):
        return self.callback_(_string)

def callback(config):
    rospy.loginfo("Config set to {inflation_radius}, {cost_scaling_factor}, {enabled}".format(**config))

def dynamic_update(req):
    print "node_name:{}, config_name:{}, new_cnofig:{}".format(req.node_name, req.config_name, req.new_cnofig)
    client = dynamic_reconfigure.client.Client(req.node_name, timeout=30, config_callback=callback)
    tc = type_converter(req.config_type)
    client.update_configuration({ req.config_name : tc.convert_(req.new_cnofig)})
    return "ok"

def server_test():
    rospy.init_node('dynamic_reconfig')
    s = rospy.Service('dynamic_update', DynamicUpdate, dynamic_update)
    print "Ready to do dynamic_update."
    rospy.spin()

if __name__ == "__main__":
    try:
        server_test()
        #~ rospy.init_node("dynamic_client", anonymous=False)

        #~ client = dynamic_reconfigure.client.Client(_name, timeout=30, config_callback=callback)
        #~ pairs = client.get_configuration()
        #~ for key, value in pairs.iteritems():
          #~ print "{} : {}".format(key, value)
        #~ client.update_configuration({ 'inflation_radius' : 20})
    #~ r = rospy.Rate(0.1)
    #~ x = 0
    #~ b = False
    #~ while not rospy.is_shutdown():
        #~ x = x+1
        #~ if x>10:
            #~ x=0
        #~ b = not b
        #~ client.update_configuration({"int_param":x, "double_param":(1/(x+1)), "str_param":str(rospy.get_rostime()), "bool_param":b, "size":1})
        #~ r.sleep()
    except rospy.ROSInterruptException:
        print "ros exception"
        pass
