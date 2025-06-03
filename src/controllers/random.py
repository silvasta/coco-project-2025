from numpy import random, clip
from src.controllers.controller import Controller

class Random(Controller):

    def __init__(self,actuators,params= {}):
        super().__init__(actuators,params)
        self.name = 'Random'
        

    def get_next_input(self):
        u = random.uniform(low=self.safety[0],high=self.safety[1],size=self.m)
        #u = random.normal(loc = self.actuators.get_uhat(),scale = 0.1, size = self.m)
        return u
