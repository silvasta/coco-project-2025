import os,sys
parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent)
from abc import ABC, abstractmethod
from tools.utils import pickler

from network.network import Network

class Task(ABC):
    """Abstract class for tasks."""

    def __init__(self, taskparams = dict):
        
        self.pickler = pickler()
        self.taskparams = taskparams
        self.id = taskparams['id']
        self.n_regions = taskparams['n_regions']
        self.taskname = taskparams['taskname']
        self.actuators = taskparams['actuators']
        self.gui = taskparams['gui']
        self.files = taskparams['files']
        self.horizon = taskparams['horizon']
        self.Tini = taskparams['Tini']
        self.begin = taskparams['begin']
        self.end = taskparams['end']
        self.scale = taskparams['scale']
        self.networkname = taskparams['networkname']
        self.labels = taskparams['labels']
        self.freq = taskparams['freq']
        self.n_threads = taskparams['n_threads']
        self.demand_path = f"./dep/sumo_files/{self.networkname}/routing/{taskparams['files']['demand']}"
        self.forecast_path = f"./dep/sumo_files/{self.networkname}/routing/{taskparams['files']['forecast']}"
        self.theta_path = None
        tasks = [file[:-3] for file in os.listdir('./src/tasks')]
        tasks.remove('task')
        if self.taskname not in tasks:
            raise ValueError(f"Task {self.taskname} not found in ./src/tasks")
        
        self.network = Network(self.networkname,self.freq,self.n_threads,self.labels, self.demand_path, self.theta_path, taskparams['n_PWA'])
        
    @abstractmethod
    def runtask(self):
        """"""
        pass