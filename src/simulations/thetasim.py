import os,sys
from simulations.simulation import Simulation
from models.theta import Theta
from src.data.data import Data

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)        
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

class ThetaSim(Simulation):

    def __init__(self,network,taskparams):
        super().__init__(network = network,taskparams=taskparams)
    
    def run(self):

        step = self.begin
        
        while step < self.begin+ self.T:    
            traci.simulationStep(step) 
            step += 1

        return None
    
    def create_theta(self):

        xmlpath = self.output_path + 'routes.xml'
        self.theta = Theta(xmlpath,self.network)
        return Data(data = self.theta.get_theta())
        