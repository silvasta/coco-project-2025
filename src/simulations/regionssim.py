import os,sys
from simulations.simulation import Simulation


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)        
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from numpy import average,zeros
from data.data import Data

class RegionsSim(Simulation):

    def __init__(self,taskparams):
        super().__init__(taskparams=taskparams)
        self.edges_encoding = self.network.get_edges_encoding()
        self.edges = self.network.get_edges()   
        self.n_edges = self.network.get_n_edges()

    def run(self):

        step = self.begin
        T = int(self.T/self.freq)
        density = zeros((self.n_edges,T))
        flow = zeros((self.n_edges,T))
        k =0
        
        while step < self.begin+ self.T:    
            traci.simulationStep(step) 
                
            for edge in self.edges:     
                flow[self.edges_encoding[edge],k] = self.edges[edge].get_flow(self.freq)
                density[self.edges_encoding[edge],k] = self.edges[edge].get_density()
            
            step += self.freq
            k += 1
    
        average = average(density,axis =1)    
        average_density = {self.edges[edge].get_id(): average[self.edges_encoding[edge]] for edge in self.edges}
    
        return Data(data=average_density)