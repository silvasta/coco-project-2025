import os,sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)        
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
import traci
from copy import deepcopy
from src.actuators.actuator import Actuator
from numpy import array

class Edge(Actuator):
    
    def __init__(self, obj,freq = 1):
        super().__init__(obj)
        self.type = 'edge'
        self.baselinefreq = freq
        self.length = self.sumo_obj.getLength() #By the get_density function, assume this is in meters. TODO: verify
        self.lane_number = self.sumo_obj.getLaneNumber()
        self.cols = ['density','flow','CO2_emission','CO_emission','HC_emission',
                'PMx_emission','NOx_emission','fuel_consumption','occupancy',
                'mean_speed','mean_length','waiting_time',
                'noise_emission','electricity_consumption']
        self.funcs = array([self.get_density,self.get_flow,traci.edge.getCO2Emission,
                      traci.edge.getCOEmission,traci.edge.getHCEmission,
                      traci.edge.getPMxEmission,traci.edge.getNOxEmission,
                      traci.edge.getFuelConsumption,traci.edge.getLastStepOccupancy,
                      traci.edge.getLastStepMeanSpeed,traci.edge.getLastStepLength,
                      traci.edge.getWaitingTime,traci.edge.getNoiseEmission,
                      traci.edge.getElectricityConsumption])
        
        self.last_step_vehicles = set()
        self.uhat = 1
        self.recover_neighbours()

    def init_params(self):

        self.params = {
            'length': self.sumo_obj.getLength(),
            'from_node': self.sumo_obj.getFromNode().getID(),
            'to_node': self.sumo_obj.getToNode().getID(),
            'neighbours': self.recover_neighbours(),
            'max_speed': self.sumo_obj.getSpeed(),
            'usafety': self.sumo_obj.getSpeed()*1.5,
            'lsafety': self.sumo_obj.getSpeed()*0.5,
        }
    
    def init_region(self,label):
        self.region = label
    
    def get_state(self,cols = []):
        
        idx = [self.cols.index(col) for col in cols]
        self.state = array([fun(self.sumoid) for fun in self.funcs[idx]])

        return self.state

    def get_uhat(self):
        return self.uhat
    
    def set_input(self,u = float):
        speed = self.params['max_speed']*u
        traci.edge.setMaxSpeed(self.sumoid,speed)

    def set_allowed(self,allowedClasses=[]):
        traci.edge.setAllowed(self.sumoid,allowedClasses)
    
    def set_disallowed(self,disallowedClasses=[]):
        traci.edge.setDisallowed(self.sumoid,disallowedClasses)

    def recover_neighbours(self):
        '''
        Finds and stores the IDs of all other edges that are connected to the start or end nodes of this edge.
        '''
        neighbours =  set()  # Initialize an empty set to store the neighbour IDs
        self.from_node = self.sumo_obj.getFromNode()  # Get the node at the start of the edge
        self.to_node = self.sumo_obj.getToNode()  # Get the node at the end of the edge
        # Add the IDs of all incoming and outgoing edges for both nodes to the neighbours set
        neighbours.update([edge.getID() for edge in self.from_node.getIncoming()] + [edge.getID() for edge in self.from_node.getOutgoing()])
        neighbours.update([edge.getID() for edge in self.to_node.getIncoming()] + [edge.getID() for edge in self.to_node.getOutgoing()])
        neighbours.remove(self.sumoid)  # Remove the ID of the current edge from the neighbours set
        self.neighbours = neighbours
          
    
    def get_density(self,sumoid):
        '''
        Calculates and returns the density of vehicles on the edge.
        '''
        accumulation = traci.edge.getLastStepVehicleNumber(sumoid)  # Get the number of vehicles on the edge
        return (accumulation/(self.length*self.lane_number))*1000  # Calculate and return the density, in vehicles per km
    
    def get_flow(self,sumoid):
        '''
        Calculates and returns the flow of vehicles on the edge.
        '''
        current_vehicles = set(traci.edge.getLastStepVehicleIDs(sumoid))  # Get the current set of vehicles on the edge
        entered = len(current_vehicles- self.last_step_vehicles)  # Calculate the number of vehicles that have entered since the last step
        self.last_step_vehicles = deepcopy(current_vehicles)  # Update last_step_vehicles to the current set of vehicles
        return (entered*3600)/self.baselinefreq  # Calculate and return the flow, in vehicles per hour

    def get_region(self):
        return self.region
    
    def get_max_speed(self):
        return self.params['max_speed']

    def get_length_km(self):
        return self.length/1000

    def get_lane_number(self):
        return self.lane_number
