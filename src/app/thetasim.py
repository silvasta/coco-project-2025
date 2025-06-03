import os,sys,json
# setting the path for the imports
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from src.controllers.tls.nocontrol import NoControl
from src.simulations.simulation import Simulation
from numpy import load,save,unique
from logs.error_handling.exceptions import *
from app.menu import SelMenu

def thetasim(mainwin,*args):
    net = args[0]
    name = args[1]
    pos = (2,0)
    if os.path.exists(f"./dep/sumo_files/{name}/regions/labels.npy"):
        labels = load(f"./dep/sumo_files/{name}/regions/labels.npy")
        n_regions = len(unique(labels))
    else:
        raise RegionLabelsError(name)
    net.init_regions(n_regions,labels)
    
    if os.path.exists(f"./dep/sumo_files/{name}/regions/density_MFD.npy") and os.path.exists(f"./dep/sumo_files/{name}/regions/flow_MFD.npy"):
        mfdden = load(f"./dep/sumo_files/{name}/regions/density_MFD.npy")
        mfdflow = load(f"./dep/sumo_files/{name}/regions/flow_MFD.npy")
        net.approximate_MFD(density=mfdden,flow=mfdflow,degree=4)
        net.recover_critical()
        net.approximate_PWA(n_PWA = 10)
    else:
        raise MFDFileError(name) 
    
    mainwin.addstr(f"Regions initialized successfully. \n \n")
    mainwin.refresh()

    if os.path.exists(f"./dep/sumo_files/{name}/control/tls/TLS.json"):
        net.init_control(f"./dep/sumo_files/{name}/control/tls/TLS.json",mode = name)   
    else: 
        raise TLSFileError(name)
    
    mainwin.addstr(f"Actuators initialized successfully. \n \n") 
    mainwin.refresh()
    params = {'r':net.get_r(1),
            'm': len(net.get_control_redlight().keys())
            }
    controller = NoControl(params=params)
    
    ### Simulation Initialization ###
    mainwin.clear()
    sims = os.listdir(f"./dep/sumo_files/{name}/simparams")
    mainwin.addstr("Please select the simulation parameters: \n")
    mainwin.refresh()
    menu = SelMenu(sims,mainwin,pos)   # Choice dependent on Control Cycle used
    simname = menu.display()
    mainwin.clear()
    mainwin.refresh()
    simparams = json.load(open(f"./dep/sumo_files/{name}/simparams/{simname}",'r'))
    sim =   Simulation(network = net,controller=controller,simparams=simparams,mode = 'Control',gui = 'False',)
    mainwin.addstr(f"Simulation initialized successfully. \n \n")   
    mainwin.refresh()

    ### Run Simulation ###
    mainwin.clear()
    mainwin.addstr(f"Simulation running... \n \n")
    sim.start_traci()
    (density_results,flow_results,input_results,error_results) = sim.run()
    sim.end_traci()

    ### Compute Theta ###