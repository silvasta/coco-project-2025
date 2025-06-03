import os,sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)        
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
import traci
from src.actuators.actuator import Actuator

class Vehicle(Actuator):

    def __init__(self, obj):
        super().__init__(obj)
        self.type = 'vehicle'

    def init_params(self):
        self.params = {
            'type': traci.vehicle.getTypeID(self.sumoid),
            'length': traci.vehicle.getLength(self.sumoid),
            'width': traci.vehicle.getWidth(self.sumoid),
            'max_speed': traci.vehicle.getMaxSpeed(self.sumoid),
            'route': traci.vehicle.getRoute(self.sumoid),
            'color': traci.vehicle.getColor(self.sumoid),
            'emission_class': traci.vehicle.getEmissionClass(self.sumoid),

        }
        self.state = {
            'speed': traci.vehicle.getSpeed(self.sumoid),
            'road': traci.vehicle.getRoadID(self.sumoid),
            'route': traci.vehicle.getRoute(self.sumoid),
            'waiting_time': traci.vehicle.getAccumulatedWaitingTime(self.sumoid),

        }

    def set_input(self, u = []):
        traci.vehicle.setRoute(self.sumoid, u)
    
    def set_routing(self,route = []):
        """
        setRoute(string, list) ->  None

        changes the vehicle route to given edges list.
        The first edge in the list has to be the one that the vehicle is at at the moment.

        example usage:
        setRoute('1', ['1', '2', '4', '6', '7'])

        this changes route for vehicle id 1 to edges 1-2-4-6-7
        """
        traci.vehicle.setRoute(self.sumoid, route)

    def set_stop(self,edgeid, duration = int):
        traci.vehicle.setStop(self.sumoid, edgeid, pos=1, duration=duration)
    
    def set_speed(self, speed = float):
        traci.vehicle.setSpeed(self.sumoid, speed)
    
    def highlight(self):
        traci.vehicle.highlight(self.sumoid, color=(255,0,0,0))
