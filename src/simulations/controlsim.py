import os,sys,json
import numpy as np
from numpy import load, empty, array, ones, concat
from simulations.simulation import Simulation
from models.linearmodel import LinearModel
from tools.utils import Parser
parent = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(parent)

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)        
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from numpy import array,zeros,eye,tile,repeat, newaxis, unique, sum
from pandas import DataFrame
from lxml import etree
from abc import ABC, abstractmethod

class ControlSim(Simulation):
        
    def __init__(self,network,taskparams,actuators,controlparams = {}):
        super().__init__(network=network,taskparams=taskparams)
        self.edges_encoding = self.network.get_edges_encoding()
        self.edges = self.network.get_edges()
        self.n_edges = self.network.get_n_edges()
        self.parser = self.parser = Parser('./src/controllers')
        self.actuators = actuators
        self.m = self.actuators.get_m()
        self.network.regions.approximate_MFD(mfddata = f"dep/sumo_files/{self.taskparams['networkname']}/regions/{self.taskparams['mfddata']}", degree = self.taskparams['degree'],n_PWA = self.taskparams['n_PWA'])
        self.n_regions = self.network.get_n_regions()
        self.cols = self.edges[0].cols
        self.variables = ['density','flow']    # This specifies what variables to retrieve from the edges among the 14 possible ones
        self.variablesIndexes = [self.cols.index(var) for var in self.variables]
        self.outputIndexes = [self.cols.index(var) for var in self.output_type]  #This governs the creation of the trajectory r and the output passed to the controller
        self.controlparams = controlparams
        self.density_columns = [f'Region {i}' for i in range(self.network.n_regions)]
        self.input_columns = actuators.encoding.keys()
        self.flow_columns =  [f'Region {i}' for i in range(self.network.n_regions)]
    
        # load the deparr.json file from self.deparr_path
        with open(self.deparr_path) as f:
            deparr = json.load(f)

        # load demand
        self.demand = load(self.demand_path)

        self.alpha = empty((self.n_regions,self.n_regions))
        for i in range(self.n_regions):
            for j in range(self.n_regions):
                self.alpha[i,j] = deparr[str(i)][str(j)]
        
        # Load linear model
        self.linear_model = self.get_model()


    def run(self, init_from_notebook= False, controller_class = None, controller_json = None):
        if (init_from_notebook):
            assert controller_class is not None and controller_json is not None, (
            "When importing controller class from notebook (init_from_notebook), both controller_class and controller_json must be provided."
        )
        self.controller = self.init_controller(init_from_notebook, controller_class, controller_json)
        u = tile(self.actuators.get_uhat(),self.m)
        y = zeros(self.n_regions*len(self.output_type))
        self.current_step = self.begin
        self.p = self.n_regions*len(self.output_type) 
        self.W = eye(self.p)
        cycle_duration = self.cycle_duration
        increment = int(cycle_duration)
        T = int(self.T/cycle_duration)
        data = zeros((self.n_regions,len(self.cols),T))
        uData = zeros((T,self.m))
        yPred = zeros((T,self.p))
        error = zeros((T,1))
        k = 0

        demand = array([sum(self.demand[:, 4, i*cycle_duration:(i+1)*cycle_duration], axis=1) for i in range(T)])
        demand = concat([demand, ones((180, self.n_regions))]) # add the demand for the last N time steps
        
        while self.current_step < self.begin+self.T:
            traci.simulationStep(self.current_step)
            
            results = self.network.get_state(byregion = True, cols = self.variables) # get state of each network region e.g. for density and flow
            data[:,self.variablesIndexes,k:(k+1)] = results[:, :, newaxis] # store results in data
            yData = data[:,self.outputIndexes[0],:]

            u_target = np.ones(self.n_regions)
            A, B, C, d = self.linear_model.linearize(cycle_duration, self.r, u_target)
            u, y = self.compute_input(k, demand, self.controller, self.controller.name,  
                                      uData, yData, yPred, self.m, self.p, self.r, self.controller.safety[0], self.controller.safety[1])

            uData[k:(k+1)] = u
            yPred[k:(k+1)] = y

            if self.controller.name != 'NoControl':
                self.actuators.set_inputs(u)

            distance = y - self.r
            error[k:(k+1)] = distance @ self.W @ distance.reshape(-1,1)
            
            self.current_step += increment
            k += 1

        #Output creation
        self.output = {f"{var}_results": DataFrame(data[:,self.cols.index(var),:].T,columns=[f'Region {i}' for i in range(self.n_regions)]) for var in self.variables}
        self.output['input_results'] = DataFrame(uData, columns=self.input_columns)
        self.output['error_results'] = DataFrame(error, columns=['Error'])
        self.pred = {f"{var}_prediction_results": DataFrame(yPred[:,self.cols.index(var)*self.n_regions:(1 + self.cols.index(var))*self.n_regions],columns=[f'Region {i}' for i in range(self.n_regions)]) for var in self.output_type}
        self.output.update(self.pred)

    @abstractmethod
    def compute_input(self):
        '''Computes the input for the controller. This function is called every cycle_duration seconds.'''
        raise NotImplementedError("compute_input method must be implemented in subclasses")

    
    def init_controller(self,init_from_notebook= False, controller_class = None, controller_json = None):
        '''Initializes the controller.'''
        if (init_from_notebook):
            class_ = controller_class
            controller_json_file = controller_json
        else:
            class_ = self.parser.import_class(self.taskparams['controller'])
            controller_json_file = f"./dep/sumo_files/{self.taskparams['networkname']}/control/{self.taskparams['actuators']}/{self.taskparams['controller']}.json"

        self.r = self.network.regions.get_r(1, mode = self.taskparams['output_type'])

        if self.controlparams == {}:
            try:
                with open(controller_json_file,'r') as f:
                    params = json.load(f)
            except:
                params = controller_json_file
        else:
            params = self.controlparams

        params['n_regions'] = self.n_regions
        # params['linear_model'] = self.get_model()
        if self.actuator_type == 'edge':
            params['default_speed_limit'] = self.actuators.actuators[0].get_max_speed()
        params['info']={'net':self.taskparams['networkname'],
                            'act':self.taskparams['actuators'],
                            }
        
        return class_(self.actuators,params = params)
    
    def get_model(self):
        linear_model_instance = LinearModel(network=self.network, demand=self.network.get_demand())
        return linear_model_instance

    def compute_metrics(self):
    
        parser = etree.XMLParser(recover=True)
        tree = etree.parse(self.output_path+"tripinfo.xml",parser= parser)
        root = tree.getroot()
        columns = ['travel_time','waiting_time','C0_abs','C02_abs','HC_abs','PMx_abs','NOx_abs','fuel_abs']
        values = zeros((len(root),len(columns)))
        i = 0 
        for trip in root:
            emissions = trip.__getitem__(0)
            values[i,:] = array([trip.values()[10],trip.values()[12],
                                emissions.values()[0],emissions.values()[1],
                                emissions.values()[2],emissions.values()[3],
                                emissions.values()[4],emissions.values()[5]]).astype(float)
            i += 1
        values =array([values[:,0]/60,values[:,1]/60,
                       values[:,2]/1000,values[:,3]/1000,
                       values[:,4]/1000,values[:,5]/1000,
                       values[:,6]/1000,values[:,7]/1000]).T
        metrics = DataFrame(values,columns=columns).mean().round(2)
        metrics['nvehicles'] = int(len(root))
        self.output['metrics'] = metrics
        
        
        return self.output

