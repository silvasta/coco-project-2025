import os,sys
from simulations.simulation import Simulation
parent = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(parent)

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)        
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from numpy import zeros
from data.data import Data

class MFDSim(Simulation):

    def __init__(self,network,taskparams):
        super().__init__(network = network,taskparams=taskparams)
        self.edges_encoding = self.network.get_edges_encoding()
        self.edges = self.network.get_edges()
        self.n_edges = self.network.get_n_edges()
        self.scale = str(2)
        self.variables = ['density','flow']

    def run(self):
        
        step = self.begin
        T = int(self.T/self.freq)                                              
        k =0
        data = zeros((self.network.n_regions,len(self.variables),T))
        while step < self.begin+ self.T:    
            traci.simulationStep(step) 
            
            results = self.network.get_state(byregion = True,cols = self.variables)
            data[:,:,k] = results
            step += self.freq
            k += 1
    
        return data
    