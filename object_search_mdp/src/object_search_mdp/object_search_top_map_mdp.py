from mdp_plan_exec.mdp import Mdp, MdpTransitionDef, MdpPropDef
from object_search_mdp.top_map_mdp import TopMapMdp

from itertools import product

class ObjectSearchTopMapMdp(TopMapMdp):
    def __init__(self,top_map_name, object_list, waypoints_matrix, probs_matrix, costs_matrix):
        TopMapMdp.__init__(self, top_map_name)
        self.object_found_prop_strings = []
        self.add_objects(object_list, waypoints_matrix, probs_matrix, costs_matrix)
        
        
        ##list of attributes of an MDP object
        #self.n_state_vars=0 #numer of variables used to defined the mdp state
        #self.state_vars=[] #names of the different variables used to define the state
        #self.state_vars_range={} #ranges for the state vars
        #self.initial_state={} #dict indexed by the state vars
        #self.n_props=0 #number of propositional labels
        #self.props=[] #list of propositional label names
        #self.props_def={} #dict of MdpPropDef instances. keys are the propositional labels names
        #self.n_actions=0 #number of actions
        #self.actions=[] #list of action names
        #self.transitions=[] #list of MdpTransitionDef instances
        #self.current_policy=[]
        #self.reward_names={}
        
    def get_object_var_name(self, object_name, waypoints_list, index):
        return object_name + '_at_' + waypoints_list[index]
        

    def add_objects(self, object_list, waypoints_matrix, probs_matrix, costs_matrix):
        for (object_name, waypoints_list, probs_list, costs_list) in zip(object_list, waypoints_matrix, probs_matrix, costs_matrix):
            prop_string = 'label "found_' + object_name + '" = ('
            perceive_object_action_name = 'perceive_' + object_name
            self.actions.append(perceive_object_action_name)
            self.n_actions=self.n_actions+1
            n_waypoints = len(waypoints_list)
            total_prob = sum(probs_list)
            current_waypoint_index=0
            for (waypoint_name, prob, cost) in zip(waypoints_list, probs_list, costs_list):               
                var_name = object_name + '_at_' + waypoint_name
                print "ADDING", var_name
                self.state_vars.append(var_name)
                self.state_vars_range[var_name]=(-1,1) #-1, unkown, 0 not found, 1 found
                self.initial_state[var_name]=-1
                waypoint_val = self.props_def[waypoint_name].conds['waypoint']
                prop_string = prop_string + var_name + '=1' + ' | '
                possible_combinations = ["".join(seq) for seq in product("01", repeat=n_waypoints-1)]
                for comb in possible_combinations:
                    transition = MdpTransitionDef(action_name = perceive_object_action_name,
                                            pre_conds={'waypoint':waypoint_val, var_name:-1},
                                            prob_post_conds=[],
                                            rewards={'time':cost},
                                            exec_count=0)
                    unk_sat_prob_sum = 0
                    not_sat_prob_sum = 0.0
                    for i in range(0, n_waypoints-1):
                        if i >= current_waypoint_index:
                            over_wapoint_index = 1
                        else:
                            over_wapoint_index = 0
                        current_waypoint_var_name = object_name + '_at_' + waypoints_list[i + over_wapoint_index]
                        if comb[i] == '0':
                            not_sat_prob_sum = not_sat_prob_sum + probs_list[i + over_wapoint_index]
                            transition.pre_conds[current_waypoint_var_name] = 0
                        else:
                            transition.pre_conds[current_waypoint_var_name] = -1
                            unk_sat_prob_sum = unk_sat_prob_sum + probs_list[i + over_wapoint_index]
                    final_prob = (prob)/(1 - not_sat_prob_sum)
                    transition.prob_post_conds=[(1-final_prob,{'waypoint':waypoint_val, var_name:0}),(final_prob,{'waypoint':waypoint_val, var_name:1})]
                    self.transitions.append(transition)
                current_waypoint_index = current_waypoint_index + 1
            prop_string = prop_string[:-3] + ');\n'
            self.object_found_prop_strings.append(prop_string)
            
                        
            
    def write_prism_model(self,file_name):
        TopMapMdp.write_prism_model(self, file_name)
        f = open(file_name,'r')
        f_aux=open(file_name + '_aux','w')
        at_labels = False
        while not at_labels:
            line=f.readline()
            f_aux.write(line)
            if 'label "' in line:
                at_labels = True
        for prop_string in self.object_found_prop_strings:
            f_aux.write(prop_string)
        for line in f:
            f_aux.write(line)
        f.close()
        f_aux.close()
        f = open(file_name,'w')
        f_aux=open(file_name + '_aux','r')
        for line in f_aux:
            f.write(line)
        f.close()
        f_aux.close()

               

        #print "add shit and set doors"
        #door_targets=[]
        #for i in range(0,len(self.transitions)):
            #transition = self.transitions[i]
            #if "doorPassing" in transition.action_name:
                ##add stuff to model
                #source=transition.pre_conds['waypoint']
                #target=transition.prob_post_conds[0][1]['waypoint']
                #if source in door_targets:
                    #index=door_targets.index(source)
                    #var_name="door_edge" + str(index)
                    #check_door_action_name='check_door' + str(index)
                #else:
                    #door_targets.append(target)
                    #var_name="door_edge"+str(self.n_door_edges)
                    #print var_name
                    #self.state_vars.append(var_name)
                    #self.initial_state[var_name]=-1
                    #check_door_action_name="check_door"+str(self.n_door_edges)
                    #self.actions.append(check_door_action_name)
                    #self.n_actions=self.n_actions+1
                    #self.n_door_edges=self.n_door_edges+1
                    ##update transition <- can only pass if door is open
                #self.transitions[i].pre_conds[var_name]=1
                #if not self.forget_doors:
                    ##reset dooo value to -1
                    #for j in range(0, len(self.transitions[i].prob_post_conds)):
                        #self.transitions[i].prob_post_conds[j][1][var_name]=-1
                ##create check transition                
                #self.transitions.append(MdpTransitionDef(action_name=check_door_action_name,
                                            #pre_conds={'waypoint':source, var_name:-1},
                                            #prob_post_conds=[(0.1,{'waypoint':source, var_name:0}),(0.9,{'waypoint':source, var_name:1})],
                                            #rewards={'time':0.1},
                                            #exec_count=0))
    