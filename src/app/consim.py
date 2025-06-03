import os,sys,ast,json,pickle
# setting the path for the imports
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from logs.error_handling.exceptions import *
from numpy import eye,unique,load
from app.menu import SelMenu
from src.controllers.tls.deepc import DeePC
from src.controllers.tls.mpc import MPC
from src.controllers.tls.nocontrol import NoControl
from src.simulations.simulation import Simulation
from src.data.experiment import Experiment

def consim(mainwin,*args):
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
    
    if os.path.exists(f"./dep/sumo_files/{name}/control/{modname}/TLS.json"):
        net.init_control(f"./dep/sumo_files/{name}/control/{modname}/TLS.json",mode = name)   
    else: 
        raise TLSFileError(name)
    mainwin.addstr(f"Actuators initialized successfully. \n \n") 
    mainwin.refresh()
    
    with open(f'./src/mods/{modname}/controller.py','r') as file:
        node = ast.parse(file.read())
    mainwin.clear()
    control = [n.name for n in node.body if isinstance(n, ast.ClassDef)] 
    control.remove('Controller')
    mainwin.addstr(f"Selected modality : {modname} \n \n")
    mainwin.addstr(f"Please select the controller to be used for {modname}:  \n")
    pos = (5,0)
    menu = SelMenu(control,mainwin,pos)
    namecon = menu.display()
    mainwin.refresh()

    if namecon not in control:
        raise ControllerNameError(namecon)
    
    try:
        if namecon == 'DeePC':
            if os.path.exists(f"./dep/sumo_files/{name}/control/{modname}/Hankel.pkl"):
                with open(f"./dep/sumo_files/{name}/control/{modname}/Hankel.pkl",'rb') as file:
                    Hankel = pickle.load(file)
            else:
                raise HankelFileError(name,modname)
            if os.path.exists(f"./dep/sumo_files/{name}/control/{modname}/params_DeePC.json"):
                with open(f"./dep/sumo_files/{name}/control/{modname}/params_DeePC.json",'r') as file:
                    params = json.load(file)
            else:
                raise ParamsFileError(name,namecon,modname)
            
            params['r'] = net.get_r(Hankel.N,mode = params['mode'])
            params['n_regions'] = n_regions
            params['Q'] = eye(1)
            params['R'] = eye(12)*0.00001
            params['H'] = Hankel
            controller = DeePC(params=params)
            
        elif namecon == 'MPC':
            controller = MPC(params={})
        elif namecon == 'NoControl':
            params = {'r':net.get_r(1),
                        'm': len(net.get_control_redlight().keys())
                        }
            controller = NoControl(params=params)
    except:
        raise ControllerCreationError(namecon)
    
    mainwin.clear()
    sims = os.listdir(f"./dep/sumo_files/{name}/simparams")
    mainwin.addstr("Please select the simulation parameters: \n")
    mainwin.refresh()
    menu = SelMenu(sims,mainwin,pos)   # Choice dependent on Control Cycle used
    simname = menu.display()
    mainwin.clear()
    mainwin.refresh()

    ### Simulation Initialization ###
    simparams = json.load(open(f"./dep/sumo_files/{name}/simparams/{simname}",'r'))
    sim =   Simulation(network = net,controller=controller,simparams=simparams,mode = 'Control', gui = simparams['gui'],)
    mainwin.addstr(f"Simulation initialized successfully. \n \n")   
    mainwin.refresh()

    ### Simulation Running ###
    mainwin.clear()
    mainwin.addstr(f"Simulation running... \n \n")
    sim.start_traci()
    net.init_programs()
    (density_results,flow_results,input_results,error_results) = sim.run()
    sim.end_traci()

    ### Experiment creation ###
    info = sim.get_info()
    exp = Experiment()
    exp.define(info = info,name = '',description = '')


   