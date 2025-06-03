from abc import ABC, abstractmethod
from tools.utils import pickler

class Controller(ABC):
    ''' Abstract controller class
    
        Args:
            params (dict): Generic dictionary of parameters
            name (str): Name of the controller
    '''
    def __init__(self,actuators,params= {}) -> None:
        '''Initialize the controller '''
        
        self.name='Controller'
        self.params = params
        self.actuators = actuators
        self.m = actuators.get_m()
        self.safety = actuators.get_safety()
        self.pickler = pickler()

        self.n_regions = params['n_regions']
        # self.r = params['r'][:self.n_regions]
        # self.actuator_type = params['actuator_type']

        self.ul = self.safety[0] # lower bound on the input
        self.uu = self.safety[1] # upper bound on the input
        
        
    ''' This method should be written in a parallelizable way ? '''
    @abstractmethod
    def get_next_input(self, *args):
        '''Get the next input.

        Args:
            args: For consistency with other controller
        Returns:
            Next inputs used in the simulation
        '''
        return None

    def get_params(self):
        return self.params