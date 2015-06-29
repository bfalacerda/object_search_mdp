#! /usr/bin/env python

import sys
import rospy
from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.product_mdp import ProductMdp
from mdp_plan_exec.prism_java_talker import PrismJavaTalker
from object_search_mdp.object_search_top_map_mdp import ObjectSearchTopMapMdp



if __name__ == '__main__':
    rospy.init_node('test_client')
    
    top_map_name=rospy.get_param("topological_map_name")
    
    object_list = ['cup', 'banana']
    waypoints_matrix = [['MaMa', 'DungeonExit', 'GhostRoom3', 'SlowDog', 'FormalCasualExit', #'ExploraDora'
                         ], ['GhostRoom3']]
    probs_matrix = [[0.1, 0.6, 0.7, 0.3, 0.3, 0,4], [0.9]]
    costs_matrix = [[11,12,13.4, 11.2, 11.2, 1], [10]]
    
    top_map_mdp=ObjectSearchTopMapMdp(top_map_name, object_list, waypoints_matrix, probs_matrix, costs_matrix)
    top_map_mdp.set_initial_state_from_waypoint("Station")
    
    #top_map_mdp.update_nav_statistics()
    top_map_mdp.write_prism_model('/opt/prism-robots-dev/prism/tests/search/teste.prism')
    
    directory = '/opt/prism-robots-dev/prism/tests/search'
  
    
    port=8087    
    prism_client=PrismJavaTalker(port, directory, "teste.prism")
    #prism_client.check_model('R{"time"}min=? [ (F "WayPoint3") & (F "WayPoint5")]')
    #prism_client.get_state_vector('R{"time"}min=? [ (F "WayPoint3") & (F "WayPoint5")]')
    #prism_client.get_policy('R{"time"}min=? [ (!"WayPoint3" U "WayPoint5")]')
    #prism_client.get_policy('test', 'R{"time"}min=? [ (F "WayPoint1") & (F "WayPoint6")]')
    #prism_client.get_policy('Pmax=? [ ((!"WayPoint22") U "WayPoint1") & ((!"WayPoint22") U "WayPoint7")  & ((!"WayPoint22") U "WayPoint21")]')
    

    #product=ProductMdp(top_map_mdp, directory+'prod.sta',directory+'prod.lab',directory+'prod.tra',directory+'prod.aut')
    #product.write_prism_model('/home/bruno/Desktop/product.prism')
    
    #prism_client.add_model('test2','/home/bruno/Desktop/product.prism')
    #prism_client.get_policy('test2', 'R{"time"}min=? [ (F ("WayPoint2" & (X "WayPoint3")))]')
    
    #product=ProductMdp(product, directory+'test2/prod.sta',directory+'test2/prod.lab',directory+'test2/prod.tra',directory+'test2/prod.aut')
    #product.write_prism_model('/home/bruno/Desktop/product2.prism')
    
    #prism_client.add_model('new_rew', '/home/bruno/Desktop/product.prism')
    #prism_client.get_policy('new_rew', 'multi(R{"time"}min=? [ (F "dra_acc_state1")], R{"goal1_rew"}max=? [ (F "dra_acc_state1")])')
    ##prism_client.get_policy('new_rew', 'multi(R{"time"}min=? [ (F "dra_acc_state1")],Pmax=? [ (F "dra_acc_state1")])')

    ##doors_top_map_mdp=DoorsTopMapMdp('lg_june14')
    ##doors_top_map_mdp.write_prism_model('/home/bruno/Desktop/doors.prism')

    
    prism_client.shutdown(False)