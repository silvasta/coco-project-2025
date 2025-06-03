import os,sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)        
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
import traci
from src.actuators.actuator import Actuator
from numpy import zeros, where, array

class TLS(Actuator):
    '''
    Traffic Light Signal Actuator

    parameters:
        state: tuple containing the current states of the lights
        Logics: list of the loaded programs for the tls

    '''
    
    def __init__(self,obj,control_cycle = {}):
        super().__init__(obj)
        self.type = 'tls'
        self.control_cycle = control_cycle

    def init_params(self):

        self.originalstate = {'state':traci.trafficlight.getRedYellowGreenState(self.sumoid),
                       'logics':traci.trafficlight.getAllProgramLogics(self.sumoid),
                       'program':traci.trafficlight.getProgram(self.sumoid),
                       'phase':traci.trafficlight.getPhase(self.sumoid),
                       'phase_duration':traci.trafficlight.getPhaseDuration(self.sumoid),
                       'nextswitch':traci.trafficlight.getNextSwitch(self.sumoid),
                       'controlledlinks':traci.trafficlight.getControlledLinks(self.sumoid),

                      }
        self.baselinefreq = sum([phase.duration for phase in self.originalstate['logics'][0].phases])
        self.init_program()

    def update_state(self):
        self.state = {'state':traci.trafficlight.getRedYellowGreenState(self.sumoid),
                'logics':traci.traci.trafficlight.getAllProgramLogics(self.sumoid),
                'program':traci.trafficlight.getProgram(self.sumoid),
                'phase':traci.trafficlight.getPhase(self.sumoid),
                'phase_duration':traci.trafficlight.getPhaseDuration(self.sumoid),
                'nextswitch':traci.trafficlight.getNextSwitch(self.sumoid),
                      }

    ''' CHECK THAT AFTER THE LOGIC IS SET THE PROGRAM GETS ADOPTED IMMEDIATELY!!!'''
    def set_input(self, u = float):  
        '''
        Sets the traffic light program based on a control parameter `u`. 
        Calculates the duration of the green and red phases, 
        verifies that the total cycle duration is unchanged, and sets the new phases and logic.
        '''
        # Calculate the duration of the green phase based on the control parameter `u`
        green = float(round(u*(self.control_cycle['cycle_duration'] - self.n_phases[2]*self.control_cycle['yellow'] -self.control_cycle['red']*self.n_phases[1])/self.n_phases[0]))  
        
        # Calculate the duration of the red phase based on the control parameter `u`
        red = float(round((self.control_cycle['cycle_duration'] - self.n_phases[2]*self.control_cycle['yellow'] -green*self.n_phases[0])/self.n_phases[1]))
        
        # Initialize a list to store the new phases
        phases = []
        
        # Assert that the total cycle duration is unchanged
        assert  green*self.n_phases[0]+red*self.n_phases[1]+self.control_cycle['yellow']*self.n_phases[2] == self.control_cycle['cycle_duration'], "Error : Changed cycle duration "
        
        # Iterate over the identified phases
        for j in range(len(self.identified_phases)):
            # If the phase is yellow, append it to the new phases with its original duration
            if self.identified_phases[j][0] == 'yellow':
                phases.append(traci.trafficlight.Phase(self.control_cycle['yellow'], self.identified_phases[j][1].state, 0, 0))
                
            # If the phase is green, append it to the new phases with the calculated green duration
            if self.identified_phases[j][0] == 'green':
                phases.append(traci.trafficlight.Phase(green, self.identified_phases[j][1].state, 0, 0))
            
            # If the phase is red, append it to the new phases with the calculated red duration
            if self.identified_phases[j][0] == 'red':
                phases.append(traci.trafficlight.Phase(red, self.identified_phases[j][1].state, 0, 0))
      
        # Set the new logic with the new phases
        self.logic = traci.trafficlight.Logic("0", 0, 0, phases = phases)
        #self.logic = traci.trafficlight.Logic("0", 0, 0, phases)
        
        # Set the new logic for the traffic light in the SUMO simulation
        traci.trafficlight.setProgramLogic(self.sumoid, self.logic)
    
    def init_program(self):
        '''
        Initializes the traffic light program. Retrieves the logic of the traffic light and its phases, calculates the cycle duration, and if the ID of the traffic light is in a specific list, it rearranges the phases and sets the new logic.
        '''
        self.phases = [phase for phase in self.originalstate['logics'][0].phases]
        self.states = [phase.state for phase in self.phases]
        self.phases_duration = [phase.duration for phase in self.phases]
        self.baselinefreq = self.control_cycle['cycle_duration']

        # GENERALIZE       
        if self.get_id() in ['C3','C4','F3','F4']:
            new_phases = [self.phases[2],self.phases[3],self.phases[0],self.phases[1]]
            self.logic = traci.trafficlight.Logic("0", 0, 0, phases = new_phases)
            traci.trafficlight.setProgramLogic(self.get_id(), self.logic)
        
        self.identify_phases()
    
    ''' Generalize this so that it works also for all cases''' 
    def identify_phases(self):
        '''
        Similar to identify_phases, but used for a grid-based traffic network where the phases are identified based on their index rather than their duration.
        '''
        # Initialize a list to store the identified phases
        self.identified_phases = list(zeros(len(self.phases)))
        
        # Define the indices for the green, red, and yellow phases, GENERALIZE THIS
        # green = [0]
        # red = [2]
        # yellow = [1,3]

        duration = [phase.__getattribute__('duration') for phase in self.phases]

        green = where(array(duration)== max(duration))[0]
        red = where(array(duration)== min(duration))[0]
        yellow = [i for i in range(len(self.phases)) if i not in green and i not in red]
        
        
        # Iterate over the phases
        i = 0
        for phase in self.phases:
            # If the phase is green, store it and its duration
            if i in green:
                self.identified_phases[i]= ('green',phase,self.phases_duration[i])
            # If the phase is yellow, store it and its duration
            elif i in yellow:
                self.identified_phases[i]= ('yellow',phase,self.phases_duration[i])
            # If the phase is red, store it and its duration
            elif i in red:
                self.identified_phases[i]= ('red',phase,self.phases_duration[i])
            
            i+=1
        self.n_phases = (len(green),len(red),len(yellow))
        self.uhat = self.phases_duration[green[0]]/self.baselinefreq
    
    def set_program(self,program = ()):
        traci.trafficlight.setProgram(self.sumoid,program)

    def set_phase(self,phase = int):
        traci.trafficlight.setPhase(self.sumoid,phase)    
    
    def get_state(self):
        return self.state
    
    def get_uhat(self):
        return self.uhat
