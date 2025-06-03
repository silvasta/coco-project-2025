from src.tasks.task import Task
from src.actuators.actuatorgroup import ActuatorGroup
from src.simulations.controlsim import ControlSim
# from src.simulations.tacontrolsim import TAControlSim
from src.data.experiment import Experiment

import shutil

class DSL(Task):
    """Dynamic Speed Limit Task"""

    def __init__(self, taskparams, StudentControlSim):
        super().__init__(taskparams=taskparams)
        self.actuators = ActuatorGroup(network=self.network,jsonfile=taskparams['files']['actuators'],actuator_type='edge')
        self.simulation = StudentControlSim(network=self.network,taskparams=self.taskparams, actuators=self.actuators)
    
    def runtask(self, init_from_notebook= False, controller_class = None, controller_json = None):
        '''Runs the task.'''
        print('Running Simulation...')
        self.simulation.start_traci()
        self.actuators.init_params()
        try:
            if (init_from_notebook):
                assert controller_class is not None and controller_json is not None, (
                "When importing controller class from notebook (init_from_notebook), both controller_class and controller_json must be provided."
            )
                self.simulation.run(init_from_notebook, controller_class, controller_json)
            else:
                self.simulation.run()
        except Exception as e:
            print(f"Error occurred during simulation: {e}")
            self.simulation.close_traci()
            # delete the output folder to avoid cluttering the disk
            shutil.rmtree(self.simulation.output_path)
            raise e
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

        # delete the output folder to avoid cluttering the disk
        # shutil.rmtree(self.experiment.output_path)

        return self.experiment
