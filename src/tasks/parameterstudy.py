from src.tasks.task import Task
from actuators.actuatorgroup import ActuatorGroup
from simulations.controlsim import ControlSim
from data.experiment import Experiment
from pandas import DataFrame
from numpy import linspace

class ParameterStudy(Task):

    def __init__(self,taskparams):
        super().__init__(taskparams=taskparams)
        self.actuators = ActuatorGroup(network=self.network,jsonfile=taskparams['files']['actuators'],cycle=taskparams['files']['control_cycle'],actuator_type=self.taskparams['actuators'])
        self.parameters = self.taskparams['parameters']
        self.paramrange = linspace(*self.taskparams['paramrange'])
        self.nparams = self.taskparams['paramrange'][2]

    def runtask(self):
        '''Runs the task.'''
        counter = 0
        params = {
                "lambda_g" : 1,
                "lambda_1" : 1,
                "lambda_ini": 0,
                "Q":1,
                "R":1,
                "solver":"OSQP",
                "noise":0,
                "uhat":"",
                "hankel":f"hankel_random.pkl"
            }
        
        print('Running Parameter Study...')
        for i in range(self.nparams):
            params[self.parameters[0]] = self.paramrange[i]
            for j in range(self.nparams):
                params[self.parameters[1]] = self.paramrange[j]
                try:
                    self.simulation = ControlSim(network=self.network,
                                                 taskparams=self.taskparams, 
                                                 actuators=self.actuators,
                                                 controlparams=params)
                    self.simulation.start_traci()
                    self.actuators.init_params()
                    self.simulation.run()
                    self.simulation.close_traci()
                    data = self.simulation.comupute_metrics()
                    
                    info = {
                        'output_path': self.simulation.output_path,
                        'taskparams': self.taskparams,
                        'results': data,
                    }
                    self.experiment = Experiment(info=info,description='testing experiment')
                    self.experiment.save()
                except:
                    print(f'Error in simulation {counter} with parameters   {self.parameters[0]} = {self.paramrange[i]} and {self.parameters[1]} = {self.paramrange[j]}')
        
        print('Study Parameter Study Successful')
    