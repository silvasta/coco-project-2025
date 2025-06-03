from src.tasks.task import Task
from actuators.actuatorgroup import ActuatorGroup
from simulations.controlsim import ControlSim
from data.experiment import Experiment

class TLS(Task):
    """TLS task."""
    
    def __init__(self, taskparams):
        super().__init__(taskparams)
        self.actuators = ActuatorGroup(network=self.network,jsonfile=taskparams['files']['actuators'],cycle=taskparams['files']['control_cycle'],actuator_type='tls')
        self.simulation = ControlSim(network=self.network,taskparams=self.taskparams, actuators=self.actuators)
        
    def runtask(self):
        '''Runs the task.'''
        print('Running Simulation...')
        self.simulation.start_traci()
        self.actuators.init_params()
        self.simulation.run()
        self.simulation.close_traci()
        data = self.simulation.compute_metrics()
        print('Simulation successful')
        info = {
            'output_path': self.simulation.output_path,
            'taskparams': self.taskparams,
            'results': data,
        }
        self.experiment = Experiment(info=info,description='testing experiment')
        self.experiment.save()
