from src.actuators.actuator import Actuator

class Inert(Actuator):
    '''
    This class represents an inert actuator, which does not perform any action.
    '''
    
    def __init__(self, name, obj) -> None:
        '''
        Constructor for the Inert class. Initializes attributes using the provided SUMO object and name.
        '''
        super().__init__(name, obj)
        self.type = 'inert'
    
    def init_params(self):
        '''
        Initializes the parameters of the inert actuator.
        '''
        self.params = {}
    
    def get_params(self):
        '''
        Returns the parameters of the inert actuator.
        '''
        return self.params
    
    def set_input(self,input):
        '''
        Sets the input of the inert actuator.
        '''
        pass