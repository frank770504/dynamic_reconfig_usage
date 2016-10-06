#!/usr/bin/env python

import roslib
import rospy

import dynamic_reconfigure.client

_name = "/move_base/global_costmap/inflation_layer"

def callback(config):
    rospy.loginfo("Config set to {inflation_radius}, {cost_scaling_factor}, {enabled}".format(**config))

if __name__ == "__main__":
    try:
        rospy.init_node("dynamic_client", anonymous=False)

        client = dynamic_reconfigure.client.Client(_name, timeout=30, config_callback=callback)
        pairs = client.get_configuration()
        for key, value in pairs.iteritems():
          print "{} : {}".format(key, value)

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
