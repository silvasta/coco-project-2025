from abc import ABC, abstractmethod


class Actuator(ABC):
    """Abstract class for actuators."""

    def __init__(self,obj):
        self.sumo_obj = obj
        self.sumoid = obj.getID()
        self.type = None
        self.baselinefreq = None
        self.params = {}
        self.state = {}
        

    @abstractmethod
    def init_params(self):
        pass

    @abstractmethod
    def set_input(self,input):
        pass
    
    def get_params(self):
        return self.params

    def set_params(self,params):
        self.params = params
    
    def get_state(self):
        return self.state
    
    def get_baselinefreq(self):
        return self.baselinefreq

    def set_baselinefreq(self,freq = int):
        self.baselinefreq = freq

    def get_id(self):
        return self.sumoid
    

