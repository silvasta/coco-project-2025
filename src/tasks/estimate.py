from src.tasks.task import Task
from simulations.mfdsim import MFDSim
from simulations.regionssim import RegionsSim
from tools.utils import pickler
from actuators.actuatorgroup import ActuatorGroup
from tools.utils import Parser
from numpy import save, concat
from pandas import DataFrame

import shutil

class Estimate(Task):
    """Estimate the parameters of a model from data."""

    def __init__(self,taskparams):
        super().__init__(taskparams=taskparams)
        self.parameter = taskparams['parameter']
        self.pickler = pickler()
        

    def runtask(self):

        if self.parameter == 'regions':
            
            if self.taskparams['clusteringalg'] is None:
                raise ValueError('clusteringalg must be specified in params.json')
            
            self.parse = Parser('./src/clustering')
            clusteringalg = self.parse.import_class(self.taskparams['clusteringalg'])

            if self.taskparams['clusteringalg'] == 'ManualSelection':
                clustering = clusteringalg(self.taskparams['regions'], self.network)
            else:
                print('Running Simulation...')
                self.simulation = RegionsSim(self.taskparams)
                self.simulation.start_traci()
                avg_density = self.simulation.run()
                self.simulation.close_traci()
                print('Regions simulation successful')
                clustering = clusteringalg(self.network,avg_density,self.taskparams['n_regions'],self.taskparams['clusteringalg'])

            self.labels = clustering.clustering()
            save(f'./dep/sumo_files/{self.networkname}/regions/labels_{self.id}.npy',self.labels)
    
        if self.parameter == 'mfd':
            print('Creating Simulation...')
            self.simulation = MFDSim(self.network,self.taskparams)
            print('Simulation Created Successfully')
            self.simulation.start_traci()
            print('Running MFD simulation...')
            data = self.simulation.run()
            self.simulation.close_traci()
            print('Approximating...')
            self.network.regions.approximate_MFD(data, degree = 4)
            print('MFD created successfully')
            # save(f'./dep/sumo_files/{self.networkname}/regions/mfddata_{self.id}.npy',data)

            self.mfd = self.network.regions.get_mfd()
            self.mfd_data = data
            self.r = self.network.regions.get_r(1, mode=['density'])

            # delete the output folder to avoid cluttering the disk
            shutil.rmtree(self.simulation.output_path)
      
        

    def init_actuators(self):
        actuators = []
        return actuators


        