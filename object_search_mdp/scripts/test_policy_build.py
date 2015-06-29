#! /usr/bin/env python

import sys
import rospy
from object_search_mdp.object_search_policy import ObjectSearchPolicy



if __name__ == '__main__':
    rospy.init_node('test_client')
    policy = ObjectSearchPolicy('/home/bruno/tmp/prism/object_search/prod.sta'
        , '/home/bruno/tmp/prism/object_search/prod.lab', 
        '/home/bruno/tmp/prism/object_search/adv.tra',
        40, 
        20)
    policy.simulate_random()