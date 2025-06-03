import os,sys
from multiprocessing.pool import ThreadPool
from numpy import zeros
from pandas import concat , DataFrame
import shutil

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)        
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")
    
import sumolib
import traci
from abc import ABC, abstractmethod

class Simulation(ABC):
    """Abstract class for simulations."""
    
    def __init__(self,network,taskparams):
        self.network = network
        self.taskparams = taskparams
        self.edges = self.network.get_edges()
        self.id = taskparams['id']
        self.gui = taskparams['gui']
        self.sumocfg_path =  f"./dep/sumo_files/{self.network.name}/configuration/{taskparams['files']['sumocfg']}"
        self.routing_path =  f"./dep/sumo_files/{self.network.name}/routing/{taskparams['files']['routing']}"
        self.demand_path = f"./dep/sumo_files/{self.network.name}/routing/{taskparams['files']['demand']}"
        self.deparr_path = taskparams['deparr']
        self.additional_files = f"{taskparams['files']['additionals']}"
        self.output_type = taskparams['output_type']
        self.begin = taskparams['begin']
        self.end = taskparams['end']
        self.scale = taskparams['scale']
        self.time_to_teleport = taskparams['time_to_teleport']
        self.freq = taskparams['freq']
        self.cycle_duration = taskparams['control_cycle']
        self.supersampling = taskparams['supersampling']
        self.superfreq = taskparams['superfreq']
        self.N = taskparams['horizon']
        self.Tini = taskparams['Tini']
        self.seed = taskparams['seed']
        self.actuator_type = taskparams['actuators']

        additional = self.additional_files.split(',')
        fulladd = [f'./dep/sumo_files/{self.network.name}/additional/{add},' for add in additional]
        self.additional_path = ''.join(fulladd)
        self.additional_path = self.additional_path[:-1]
        self.T = int(self.end)-int(self.begin)

        if self.taskparams['taskname'] != 'parameterstudy':
            self.output_path = f'./out/{self.network.name}/{self.id}/'
        else:
            self.output_path = f'./out/{self.network.name}/{self.id}/simulation 0/'

    
    def generate_additional_edge_data(self):
        """Dynamically generates additional_edge_data.xml and appends it to self.additional_path."""

        # Ensure the output directory exists
        os.makedirs(self.output_path, exist_ok=True)

        # Define file paths dynamically
        additional_file_path = os.path.join(self.output_path, "additional_edge_data.xml")
        # edge_data_file_path = os.path.join(self.output_path, "edge_data.xml")
        edge_data_file_path = "edge_data_new.xml"

        # Define the XML content with the correct file reference and period
        # additional_file_content = f"""<additional>
        # <edgeData id="density_measurement" file="{edge_data_file_path}" period="10"/>
    # </additional>"""
        additional_file_content = f"""<additional>
        <edgeData id="density_measurement" file="{edge_data_file_path}" period="36"/> </additional>"""
        ## hard-coded frequency

        # Write the additional XML file
        with open(additional_file_path, "w") as file:
            file.write(additional_file_content)

        # print(f"Generated additional file: {additional_file_path}")

        # Append the newly created file path to self.additional_path
        self.additional_path = additional_file_path
        # print(f"Current self.additional_path is: {self.additional_path}")

        return additional_file_path  # Return the generated path
        
    def start_traci(self):
        '''This method starts the SUMO simulation.
        '''
        if self.gui:
            sumoBinary = sumolib.checkBinary('sumo-gui')
        else:
            sumoBinary = sumolib.checkBinary('sumo')
        cores = os.cpu_count()
        try:
            os.makedirs(self.output_path)
        except FileExistsError:
            # Folder exists — clear its contents
            for filename in os.listdir(self.output_path):
                file_path = os.path.join(self.output_path, filename)
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            # counter = 1  
            # while True:
            #     if self.taskparams['taskname'] != 'parameterstudy':
            #         try:
            #             self.output_path = f'./out/{self.network.name}/{self.id} ({counter})/'
            #             os.makedirs(self.output_path)
            #             break
            #         except FileExistsError:
            #             counter += 1
            #             pass
            #     else:
            #         try:
            #             self.output_path = f'./out/{self.network.name}/{self.id}/simulation {counter}/'
            #             os.makedirs(self.output_path)
            #             break
            #         except FileExistsError:
            #             counter += 1
            #             pass
                
        self.generate_additional_edge_data()
        if self.additional_path == '': 
            traci.start([sumoBinary,'-c', self.sumocfg_path,
                        '--route-files',self.routing_path,
                        # '--threads',str(cores),
                        '--threads', str(1), # single-thread ensures deterministic runs
                        '--seed',str(self.seed),
                        '--tripinfo-output',f'{self.output_path}/tripinfo.xml',
                        '--tripinfo-output.write-unfinished','true',
                        '--vehroute-output',f'{self.output_path}/routes.xml',
                        '--emission-output',f'{self.output_path}/emission.xml', # This one can get big.
                        '--device.emissions.period','3',
                        '--begin',str(self.begin),
                        '--end',str(self.end),
                        '--max-num-teleports', '0',
                        '--time-to-teleport',str(self.time_to_teleport),
                        '--scale',str(self.scale),
                        # '--device.rerouting.threads',str(cores),
                        '--device.rerouting.threads', str(1), # single-thread ensures deterministic runs
                        ])    
        else:        
            traci.start([sumoBinary,'-c', self.sumocfg_path,
                        '--route-files',self.routing_path,
                        '--additional-files',self.additional_path,
                        # '--threads',str(cores),
                        '--threads', str(1), # single-thread ensures deterministic runs
                        '--seed',str(self.seed),
                        '--tripinfo-output',f'{self.output_path}/tripinfo.xml',
                        '--tripinfo-output.write-unfinished','true',
                        '--vehroute-output',f'{self.output_path}/routes.xml',
                        '--emission-output',f'{self.output_path}/emission.xml',
                        '--device.emissions.period','3',
                        '--begin',str(self.begin),
                        '--end',str(self.end),
                        '--max-num-teleports', '0',
                        '--time-to-teleport',str(self.time_to_teleport),
                        '--scale',str(self.scale),
                        # '--device.rerouting.threads',str(cores),
                        '--device.rerouting.threads', str(1), # single-thread ensures deterministic runs
                        ])
        
    def close_traci(self):
        '''This method ends the SUMO simulation.'''
        try:
            os.remove(f'{self.output_path}/emission.xml') # The emission.xml is removed because it weights too much, the info is contained in tripinfo.xml
        except:
            pass
        traci.close()

    @abstractmethod
    def run(self):
        """Runs the simulation."""
        pass
