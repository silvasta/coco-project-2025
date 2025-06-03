import os,sys,json
# setting the path for the imports
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from src.controllers.controller import Controller
from src.simulations.simulation import Simulation
from numpy import save

def mfdsim(mainwin,*args):
    net = args[0]
    name = args[1]
    params = {}
    controller = Controller(params=params)
    simparams = json.load(open(f"./dep/sumo_files/{name}/simparams/mfdsim.json",'r'))
    simparams['scale'] = 5

    sim = Simulation(network = net,controller=controller,simparams=simparams,mode = 'Regions',gui = 'False',)
    
    ### Run Simulation ###
    sim.start_traci()
    average_density, density, flow = sim.run()
    sim.end_traci()

    ### Saving density, flow fro MFD ###
    save(f"./dep/sumo_files/{name}/regions/density_MFD.npy",density)
    save(f"./dep/sumo_files/{name}/regions/flow_MFD.npy",flow)