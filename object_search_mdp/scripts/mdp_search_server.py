#! /usr/bin/env python
import os
import sys
import rospy

from object_search_mdp.object_search_top_map_mdp import ObjectSearchTopMapMdp
from object_search_mdp.object_search_policy import ObjectSearchPolicy
from object_search_mdp.prism_java_talker import PrismJavaTalker

from std_msgs.msg import String
from actionlib import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from strands_navigation_msgs.msg import NavRoute, ExecutePolicyModeAction, ExecutePolicyModeFeedback, ExecutePolicyModeGoal
from object_search_mdp.msg import ObjectSearchMdpAction
from object_search_action.msg import ObjectSearchAction, ObjectSearchGoal

from soma_pcl_segmentation.srv import GetProbabilityAtWaypoint 

from random import choice
from yaml import load
   
class MdpSearchServer(object):
    def __init__(self,top_map):   
        got_service=False
        while not got_service:
            try:
                rospy.wait_for_service('soma_probability_at_waypoint', 1)
                got_service=True
            except rospy.ROSException,e:
                rospy.loginfo("Waiting for soma_probability_at_waypoint service...")
            if rospy.is_shutdown():
                return       
        self.get_probs_costs_srv=rospy.ServiceProxy("soma_probability_at_waypoint", GetProbabilityAtWaypoint)
        
        self.top_nav_policy_exec= SimpleActionClient('/topological_navigation/execute_policy_mode', ExecutePolicyModeAction)
        got_server=self.top_nav_policy_exec.wait_for_server(rospy.Duration(1))
        self.object_search_ac = SimpleActionClient('/search_object', ObjectSearchAction)
        got_server = got_server and self.object_search_ac.wait_for_server(rospy.Duration(1))
        while not got_server:
            got_server=self.top_nav_policy_exec.wait_for_server(rospy.Duration(1))
            if not got_server:
                rospy.loginfo("Waiting for topological navigation execute policy mode action server.")
            #got_server = got_server and self.object_search_ac.wait_for_server(rospy.Duration(1))
            #if not got_server:
                #rospy.loginfo("Waiting for sampling based local object search action.")
            if rospy.is_shutdown():
                return
        
        self.directory = os.path.expanduser("~") + '/tmp/prism/object_search/'
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.file_name=top_map+".mdp"
        self.prism_policy_generator=PrismJavaTalker(8087,self.directory, self.file_name)
        
        self.current_prod_state=None
        self.product_mdp=None
        

               
        self.current_waypoint=None
        self.closest_waypoint=None
        self.current_waypoint_sub=rospy.Subscriber("/current_node", String, self.current_waypoint_cb)
        self.closest_waypoint_sub=rospy.Subscriber("/closest_node", String, self.closest_waypoint_cb)
        
        self.policy_mode_pub=rospy.Publisher("/mdp_plan_exec/current_policy_mode", NavRoute,queue_size=1)
        
        self.mdp_nav_as=SimpleActionServer('object_search_mdp', ObjectSearchMdpAction, execute_cb = self.execute_policy_cb, auto_start = False)
        self.mdp_nav_as.register_preempt_callback(self.preempt_policy_execution_cb)
        self.mdp_nav_as.start()
        
        rospy.loginfo("MDP search initialised.")

    def current_waypoint_cb(self,msg):
        self.current_waypoint=msg.data
        
    def closest_waypoint_cb(self,msg):
        self.closest_waypoint=msg.data


    

    def execute_policy_cb(self,goal):
        object_list = load(goal.objects)
        waypoints_list = load(goal.waypoints)
        probs_costs_srv_res = self.get_probs_costs_srv(waypoints_list, object_list)
        n_objects = len(object_list)
        n_waypoints = len(waypoints_list)
        waypoints_matrix = []
        probs_matrix = []
        costs_matrix = []
        probs_list = []
        costs_list = []
        for i in range(0, n_objects):
            for j in range(0, n_waypoints):
                probs_list.append(probs_costs_srv_res.probability[j*n_objects+i])
                costs_list.append(probs_costs_srv_res.cost[j*n_objects+i])
            waypoints_matrix.append(waypoints_list)
            probs_matrix.append(probs_list)
            costs_matrix.append(costs_list)
        self.object_search_mdp = ObjectSearchTopMapMdp(top_map_name, object_list, waypoints_matrix, probs_matrix, costs_matrix)
        self.object_search_mdp.set_initial_state_from_waypoint(self.closest_waypoint)
        current_waypoint_val = self.object_search_mdp.initial_state['waypoint']
        self.object_search_mdp.set_mdp_action_durations(self.directory+self.file_name,rospy.Time.now())
        
        ltl_spec=''
        if ltl_spec == '': 
            ltl_spec ='ijcai(Pmax=? [ (' 
            for object_name in object_list:
                ltl_spec = ltl_spec + '(F "found_' + object_name + '") & '            
            ltl_spec = ltl_spec[:-3] + ') ])'
        else:
            ltl_spec = 'ijcai(Pmax=? [ (' + goal.ltl_spec + ') ])'

        rospy.loginfo("Creating high level search policy...")
        expected_time=float(self.prism_policy_generator.get_policy(ltl_spec))
        rospy.loginfo("Policy created")
        self.policy = ObjectSearchPolicy(self.directory + '/prod.sta', 
                                    self.directory + '/prod.lab', 
                                    self.directory + '/adv.tra',
                                    self.object_search_mdp.state_vars_range['waypoint'][1] + 1,
                                    current_waypoint_val)
        current_nav_policy = self.policy.get_current_nav_policy()
        #self.current_state_description = self.flat_states[self.object_search_policy.current_state]
        while True:
            #check for execution of perceive action
            next_action = self.policy.actions[self.policy.current_state]
            while 'perceive_' in next_action:
                self.execute_perceive(next_action)
                next_action = self.policy.actions[self.policy.current_state]
                
            #check if there's more places to look
            current_nav_policy = self.policy.get_current_nav_policy()
            print current_nav_policy
            if current_nav_policy['sources'] == [] or current_nav_policy['sources'][0] == '':
                break
            
            #execute nav policy
            rospy.loginfo('Executing navigation to next search node')
            nav_policy_goal = NavRoute(source = current_nav_policy['sources'], edge_id = current_nav_policy['actions'])
            self.policy_mode_pub.publish(nav_policy_goal)
            self.top_nav_policy_exec.send_goal(ExecutePolicyModeGoal(route = nav_policy_goal), feedback_cb = self.top_nav_feedback_cb)
            self.top_nav_policy_exec.wait_for_result()
            status=self.top_nav_policy_exec.get_state()  
            rospy.loginfo("Topological navigation execute policy action server exited with status: " + GoalStatus.to_string(status))
            if status!=GoalStatus.SUCCEEDED:
                if status==GoalStatus.ABORTED:
                    self.mdp_nav_as.set_aborted()
                elif status==GoalStatus.PREEMPTED:
                    self.mdp_nav_as.set_preempted()
                else:
                    rospy.logwarn("Unexpected outcome from the topological navigaton execute policy action server. Setting as aborted")
                    self.mdp_nav_as.set_aborted()
                return
        rospy.loginfo("Object search over")
        self.mdp_nav_as.set_succeeded()
        
    def execute_perceive(self, action_name):
        object_name = ''
        object_split = action_name.split('_')
        for word in object_split[1:]:
            object_name = object_name + word + '_'
        object_name =object_name[:-1]
        #robot
        rois = {"FoodStation":"6", "GhostKitchen":"4", "Room102":"7", "Room106Exit":"2", "GhostRoom2":"3", "GhostRoom1":"5"}
        rospy.loginfo("Executing local search for " + object_name)
        search_goal = ObjectSearchGoal(waypoint = self.closest_waypoint,
                                    roi_id = rois[self.closest_waypoint],
                                    objects = [object_name])
        self.object_search_ac.send_goal(search_goal)
        self.object_search_ac.wait_for_result()
        found_objects = self.object_search_ac.get_result().found_objects
        
        ##sim
        #rospy.loginfo("Executing SIM search for " + object_name)
        #rois = {"WayPoint31":2, "WayPoint27":3, "WayPoint42":5}
        #found_objects = ['cup1', 'mug1']
        #rospy.sleep(3)
        
        possible_next_states = self.policy.next_states[self.policy.current_state]
        for state in possible_next_states:
            for prop in self.policy.flat_states[state]:
                if object_name in prop and self.policy.flat_states[state][prop] == 1:
                    success_next_state = state
                    break
        for state in possible_next_states:
            if state != success_next_state:
                failure_next_state = state
        
        if object_name in found_objects:
            self.policy.current_state = success_next_state
            rospy.loginfo("Found " + object_name)
        else:
            self.policy.current_state = failure_next_state
            rospy.loginfo("Didn't find " + object_name)
            



    def execute_perceive_sim(self, action_name):    
        print "PERCEIVE!! ", action_name
        rospy.sleep(2)
        self.policy.current_state = choice(self.policy.next_states[self.policy.current_state])
        object_name =action_name.strip('perceive_')
        prop_name =  'found_' + object_name
        for prop in self.policy.flat_states[self.policy.current_state]:
            if object_name in prop and self.policy.flat_states[self.policy.current_state][prop] == 1:
                print "FOUND ", object_name
                return 
        print "DIDNT FIND ", object_name


    def preempt_policy_execution_cb(self):     
        self.top_nav_policy_exec.cancel_all_goals()
    
    def top_nav_feedback_cb(self,feedback):
        rospy.loginfo("Reached waypoint " + feedback.route_status)
        self.policy.update_state(self.object_search_mdp.props_def[feedback.route_status].conds['waypoint'])
        
     
    def main(self):
        # Wait for control-c
        rospy.spin()       
        if rospy.is_shutdown():
            self.prism_policy_generator.shutdown(False)

if __name__ == '__main__':
    rospy.init_node('mdp_search_object')
    top_map_name=rospy.get_param("/topological_map_name")
    mdp_search =  MdpSearchServer(top_map_name)
    mdp_search.main()
    
    
