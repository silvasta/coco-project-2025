import os,sys,json,pickle
# setting the path for the imports
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from logs.error_handling.exceptions import *
from numpy import eye,unique,load,vstack
from app.menu import SelMenu
from src.controllers.tls.nocontrol import NoControl
from src.simulations.simulation import Simulation
from src.data import Hankel

def hansim(mainwin,*args):
    net = args[0]
    name = args[1]
    pos = (2,0)
    mods = os.listdir("./src/mods")
    mainwin.clear()
    mainwin.addstr("Please select the modality of control: \n \n")
    mainwin.refresh()
    menu = SelMenu(mods, mainwin, pos)
    modname = menu.display()
    mainwin.clear()

    ### Control Initialization ###
    if os.path.exists(f"./dep/sumo_files/{name}/regions/labels.npy"):
        labels = load(f"./dep/sumo_files/{name}/regions/labels.npy")
        n_regions = len(unique(labels))
    else:
        raise RegionLabelsError(name,modname)
    
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
    
    if os.path.exists(f"./dep/sumo_files/{name}/control/{modname}/TLS.json"):
        net.init_control(f"./dep/sumo_files/{name}/control/{modname}/TLS.json",mode = name)   
    else: 
        raise TLSFileError(name)
    mainwin.addstr(f"Actuators initialized successfully. \n \n") 
    mainwin.refresh()

    mainwin.addstr("Do you want to use supersampling? \n")
    mainwin.refresh()
    menu = SelMenu(['yes','no'],mainwin,pos)
    sup = menu.display()
    sup = True if sup == 'yes' else False
    mainwin.clear()
    mainwin.refresh()
    if sup:
        per = net.get_control_cycle()["cycle_duration"]
        mainwin.addstr("Please select the supersampling frequency: \n")
        mainwin.refresh()
        menu = SelMenu([per/30,per/15,per/3],mainwin,pos)   # Choice dependent on Control Cycle used
        supfreq = int(menu.display())
        mainwin.clear()
        mainwin.refresh()
    
    params = {'r':net.get_r(1),
              'm': len(net.get_control_redlight().keys())
              }
    controller = NoControl(params=params)

    sims = os.listdir(f"./dep/sumo_files/{name}/simparams")
    mainwin.addstr("Please select the simulation parameters: \n")
    mainwin.refresh()
    menu = SelMenu(sims,mainwin,pos)   # Choice dependent on Control Cycle used
    simname = menu.display()
    mainwin.clear()
    mainwin.refresh()
    simparams = json.load(open(f"./dep/sumo_files/{name}/simparams/{simname}",'r'))
    sim = Simulation(network = net,controller=controller,simparams=simparams,mode = 'Hankel',gui = 'False',)
    
    ### Running Simulation ###
    sim.start_traci()
    (density,flow,uData) = sim.run(supersampling=sup, supersampling_freq=supfreq)
    sim.end_traci()
    hankel_params = json.load(open(f"./dep/sumo_files/{name}/control/{modname}/conparams.json",'r'))

    ### Creating Hankel ###
    if simparams['output_type'] == 'density':
        hankel_params = {'uData': uData,
                         'yData': density,
                         'n': len(unique(labels)),
                         }
    elif simparams['output_type'] == 'flow':
        hankel_params = {'uData': uData,
                         'yData': flow,
                         'n': len(unique(labels)),
                         }
    elif simparams['output_type'] == 'double':
        hankel_params = {'uData': uData,
                         'yData': vstack([density,flow]),   # Fix this
                         'n': len(unique(labels)),
                         }
    
    hankel = Hankel(params= hankel_params)

    ### Saving Hankel ###

    with open(f"./dep/sumo_files/{name}/control/{modname}/Hankel_{hankel_params['Tini']}_{hankel_params['N']}.pkl",'wb') as file:
        pickle.dump(hankel,file)
