import json
from numpy import array,multiply,zeros
from src.actuators.edge import Edge
from src.actuators.inert import Inert
from src.actuators.tls import TLS
from src.actuators.vehicle import Vehicle
from tools.utils import all_equal

class ActuatorGroup():
    
    def __init__(self,network,actuator_type,jsonfile = '',cycle = None,selection = False):
        self.network = network
        self.control_matrix = None
        self.cycle = cycle  

        if selection: 
            with open(f'dep/sumo_files/{network.name}/selection/{actuator_type}/{actuator_type}All.json') as f:
                self.jsonfile = json.load(f)
        else: 
            with open(f'dep/sumo_files/{network.name}/control/{actuator_type}/'+jsonfile) as f:
                self.jsonfile = json.load(f)

        # This can be written better
        if actuator_type == 'tls':
            if cycle is not None:
                with open(f'dep/sumo_files/{network.name}/control/{actuator_type}/{self.cycle}') as f:
                    self.control_cycle = json.load(f)
            self.init_tls(self.jsonfile)
            self.lsafety = 0.6  # Grid 0.4
            self.usafety = 1
        if actuator_type == 'vehicle':
            self.init_vehicle(self.jsonfile)
            self.lsafety = 0
            self.usafety = 0
        if actuator_type == 'inert':
            self.init_inert(self.jsonfile)
        if actuator_type == 'edge':
            self.init_edges(self.jsonfile)
            self.lsafety = 0.5
            self.usafety = 1.5

    def init_params(self):
        for actuator in self.actuators:
            actuator.init_params()

        for disturbance in self.disturbances:
            disturbance.init_params()
            
    def init_tls(self,jsonfile):
        redlights = self.network.get_tls()
        self.actuators = [TLS(tls,self.control_cycle) for tls in redlights if tls.getID() in jsonfile['actuators']]
        self.encoding = {actuator.get_id():i for i,actuator in enumerate(self.actuators)}
    
    def init_edges(self,jsonfile):
        edges = self.network.get_edges()
        self.actuators = [edge for edge in edges if edge.get_id() in jsonfile['actuators']]
        self.encoding = {actuator.get_id():i for i,actuator in enumerate(self.actuators)}

        self.disturbances = [edge for edge in edges if edge.get_id() in jsonfile['disturbances']]
        self.encoding_disturbances = {actuator.get_id():i for i,actuator in enumerate(self.disturbances)}

    def init_vehicle(self,jsonfile):
        pass

    def init_inert(self):
        self.actuators = [Inert()]
        self.encoding = {actuator.get_id():i for i,actuator in enumerate(self.actuators)}
    
    def set_inputs(self,inputs,mapping = {}):
        n_regions = self.network.get_n_regions()
        if mapping == {}:
            for i,actuator in enumerate(self.actuators):
                actuator.set_input(inputs[i])
        else:
            control_matrix = self.control_matrix(mapping)
            u = u.reshape(n_regions,n_regions)
            u_given={}
            post_u = multiply(u,control_matrix)
            
            for i in range(self.n_regions):
                for j in range(self.n_regions):
                    if mapping[str(i)][str(j)] != []:
                        for r in mapping[str(i)][str(j)]:
                            u_given[r]= post_u[i,j]
                            self.redlights[r].set_program(post_u[i,j])

    def set_inputs_disturbances(self,inputs):
        for i,actuator in enumerate(self.disturbances):
            actuator.set_input(inputs[i])
    
    def control_matrix(self,mapping):
        ''' builds the control matrix input used in MPC

            Args:
                mapping: a map between the input found by MPC
                        and the actuators
        '''
        if self.control_matrix is not None:
            return self.control_matrix
        else:
            self.mapping = mapping
            control_martrix = zeros((self.n_regions,self.n_regions))
            
            for i in range(self.n_regions):
                for j in range(self.n_regions):              
                    if mapping[str(i)][str(j)] != []:
                        control_martrix[i,j] = 1

            self.control_matrix = control_martrix
            return control_martrix
    
    def get_params(self):
        return {f"{actuator.get_id()}" : actuator.get_params() for actuator in self.actuators}
    
    def get_safety(self):
        return (self.lsafety,self.usafety)
    
    def get_baselinefreq(self):
        freqs = array([actuator.get_baselinefreq() for actuator in self.actuators])
        eq = all_equal(freqs)
        return (eq,freqs)
    
    def set_baselinefreq(self,freq = int):
        for actuator in self.actuators:
            actuator.set_baselinefreq(freq)
    
    def get_m(self):
        return len(self.actuators)
    
    def get_m_disturbances(self):
        return len(self.disturbances)
    
    def get_uhat(self):
        ratios = array([actuator.get_uhat() for actuator in self.actuators])
        ratios = ratios.mean()
        return ratios
    

''' Not sure if this is needed


    def init_control(self,TLS_json ,mode = 'grid8by8'):
         Initialize control_redlight attribute

            Args:
                TLS_json: path to the TLS_json.json that
                        specifies the ids of the actuators
                mode: either 'Zurich_Stadt', 'grid', 'Perimeter'
                    Perimeter identifies by itself the actuators
                    as the ones on the boarder of the regions
    

        self.control_redlight = {}
        f = open(TLS_json)
        if mode == 'ZRH_ASP' or mode == 'ZRH_MSP':    
            layers = json.load(f)
            
            for i in layers['first_layer']:
                self.control_redlight[layers['first_layer'][str(i)]]=self.redlights[layers['first_layer'][str(i)]]
            for i in layers['second_layer']:
                self.control_redlight[layers['second_layer'][str(i)]]=self.redlights[layers['second_layer'][str(i)]]
                 
        if mode == 'grid8by8':
            
            direction = json.load(f)
            
            for dir in direction:
                for tls in direction[dir]:
                    self.control_redlight[tls]=self.redlights[tls]
            
        if mode == 'Perimeter':
            
            self.perimeter = {}            
            
            for node in self.nodes:       
                edge_neighbours = self.nodes[node].get_edge_neighbours()
                regions = np.array([self.edges[edge].get_region() for edge in edge_neighbours])
                
                if all_equal(regions):
                    self.nodes[node].is_perimeter(0)
                    
                else:
                    self.nodes[node].is_perimeter(1)
                    self.perimeter[node]= self.nodes[node]
                    
                    if self.sumo_obj.getNode(node).getType() == 'traffic_light':
                        for redlight in self.redlights:
                            #Doesn't work fix
                            if self.redlights[redlight] not in list(self.control_redlight.keys()) and node in self.redlights[redlight].get_nodes(): 
                                self.control_redlight[redlight] = self.redlights[redlight] 
            
        self.m = len(self.control_redlight)
'''