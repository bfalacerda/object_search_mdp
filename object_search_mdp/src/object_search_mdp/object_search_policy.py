from mdp_plan_exec.mdp import Mdp, MdpTransitionDef, MdpPropDef
from mdp_plan_exec.top_map_mdp import TopMapMdp

from random import choice
from Queue import Queue


class ObjectSearchPolicy(TopMapMdp):
    def __init__(self, states_file, labels_file, policy_file, n_waypoints, current_waypoint_val):
        Mdp.__init__(self)               
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
        
        self.n_waypoints = n_waypoints
        self.n_da_states=0
        self.final_automaton_state=0
        self.flat_states = []
        self.n_flat_states = 0
        self.read_states(states_file, current_waypoint_val)
        self.read_transitions(policy_file)
        self.initial_flat_state = self.get_initial_state(labels_file)
        self.current_state = self.initial_flat_state
        #self.simulate_random()
        #self.get_current_nav_policy()
        
    def update_state(self, waypoint_val):
        possible_next_states = self.next_states[self.current_state]
        for state in possible_next_states:
            if self.flat_states[state]['waypoint'] == waypoint_val:
                self.current_state = state
                return
        print "Jumped a state!"
        new_state = dict(self.flat_states[self.current_state])
        new_state['waypoint'] = waypoint_val
        for i in range(0, len(self.flat_states)):
            if self.flat_states[i] == new_state: 
                self.current_state = i
                print "Updated MDP state correctly"
                return
        print "Update mdp state error!"
        
    def get_nav_policy_target(self):
        state = self.current_state
        action = 'null'
        while True:
            if 'perceive_' in self.actions[state]:
                return action.split('_')[1]          
            action = self.actions[state]
            state = choice(self.next_states[state])
                                       
    
    def get_current_nav_policy(self):
        sources = []
        actions = []
        #targets = [[] for i in range(0, self.n_flat_states)]
        #reached = [False for i in range(0, self.n_flat_states)]
        #q = Queue()
        #q.put(self.current_state)
        #reached[self.current_state] = True
        i = 0
        if 'perceive_' not in self.actions[self.current_state] and self.flat_states[self.current_state]['_da'] != self.final_automaton_state:
            for state in self.flat_states:
                if self.compare_policy_mode(self.flat_states[self.current_state], state):
                    action = self.actions[i]
                    if 'perceive_' not in action:
                        sources.append(action.split('_')[0])
                        actions.append(action)
                        #targets[self.current_state] = list(self.next_states[self.current_state])
                i = i+1
            
        #while not q.empty():
            #current_state = q.get()
            #action = self.actions[current_state]
            #if 'perceive_' not in action:
                #sources.append(action.split('_')[0])
                #actions.append(action)           
                #for target in self.next_states[current_state]:
                    #if not reached[target]:
                        #q.put(target)
                        #reached[target] = True
                    #targets[current_state].append(target)       
        return {'sources': sources, 'actions':actions}#, 'targets': targets}        

    def compare_policy_mode(self,state1, state2):
        for key in state1:
            if key != 'waypoint':
                if state1[key]!=state2[key]:
                    return False
        return True
        
    
        
    def simulate_random(self):
        current_state = self.initial_flat_state
        while True:
            action = self.actions[current_state]
            print action
            if action == '':
                return
            current_state = choice(self.next_states[current_state])
            
        
        
    def read_transitions(self, policy_file):
        f = open(policy_file, 'r')
        self.actions = ['' for i in range(0, self.n_flat_states)]
        self.probs = [[] for i in range(0, self.n_flat_states)]
        self.next_states = [[] for i in range(0, self.n_flat_states)]
        f.readline()
        
        for line in f:
            line_array = line.split(' ')
            line_array[-1] = line_array[-1].strip('\n')
            source = int(line_array[0])
            target = int(line_array[1])
            prob = float(line_array[2])
            action = line_array[3]
            self.actions[source] = action
            self.next_states[source].append(target)
            self.probs[source].append(prob)
        f.close()
        
    def get_initial_state(self, labels_file):
        f = open(labels_file, 'r')
        f.readline()
        found = False
        for line in f:
            line_array = line.split(' ')
            if line_array[-1] == '1\n':
                final_state_index = int(line_array[0].strip(':'))
                found = True
                break
        f.close()
        if found:
            self.final_automaton_state = self.flat_states[final_state_index]['_da']
        else:
            self.final_automaton_state = 1
        
        if self.final_automaton_state == 0:
            initial_automaton_state = self.n_da_states
        elif self.final_automaton_state == self.n_da_states:
            initial_automaton_state = 0
        else:
            print "ISSUE WITH INITIAL AUTOMATON STATE"
        self.initial_state['_da'] = initial_automaton_state
        flat_state = 0
        for state in self.flat_states:
            if state == self.initial_state:
                return flat_state
            flat_state = flat_state + 1
   
        
    def read_states(self, states_file, current_waypoint_val):
        f = open(states_file, 'r')
        
        variables = f.readline()
        variables = variables.split(',')
        variables[0] = variables[0].strip('(')
        variables[-1] = variables[-1].strip(')\n')
        self.n_state_vars = 0
        for variable in variables:
            self.n_state_vars = self.n_state_vars + 1
            self.state_vars.append(variable)
            if variable == 'waypoint':
                self.state_vars_range[variable]=[0, self.n_waypoints-1]
                self.initial_state[variable] = current_waypoint_val
            if '_at_' in variable:
                self.state_vars_range[variable]=[-1, 1]
                self.initial_state[variable] = -1
        
        for line in f:
            states_string = line.split(':')
            if int(states_string[0]) != self.n_flat_states:
                print "ERROR READING POLICY STATES"
            flat_state_list = states_string[1].split(',')
            flat_state_list[0] = flat_state_list[0].strip('(')
            flat_state_list[-1] = flat_state_list[-1].strip(')\n')
            flat_state_dict={}
            for (var_name, value) in zip(self.state_vars, flat_state_list):
                flat_state_dict[var_name] = int(value)
                if var_name == '_da':
                    self.n_da_states = max(self.n_da_states, int(value))
            self.flat_states.append(flat_state_dict)
            self.n_flat_states = self.n_flat_states + 1
        self.state_vars_range['_da']=[0, self.n_da_states]
        f.close()
        
        
    def get_object_var_name(self, object_name, waypoints_list, index):
        return object_name + '_at_' + waypoints_list[index]
        

    