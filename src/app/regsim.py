import os,sys,json
# setting the path for the imports
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from src.simulations.simulation import Simulation
from src.controllers.controller import Controller
from src.data import Snake_Clustering
from app.menu import read_str,SelMenu
from numpy import save

def regsim(mainwin,*args):
    net = args[0]
    name = args[1]
    pos = (2,0)
    params = {}
    controller = Controller(params=params)
    sims = os.listdir(f"./dep/sumo_files/{name}/simparams")
    mainwin.addstr("Please select the simulation parameters: \n")
    mainwin.refresh()
    menu = SelMenu(sims,mainwin,pos)   # Choice dependent on Control Cycle used
    simname = menu.display()
    mainwin.clear()
    mainwin.refresh()
    simparams = json.load(open(f"./dep/sumo_files/{name}/simparams/{simname}",'r'))
    
    ### Asking number of regions ###
    mainwin.clear()
    mainwin.refresh()
    mainwin.addstr("How many regions do you want to create? \n \n")
    mainwin.addstr("Please enter the number of region (integer): ")
    mainwin.refresh()
    winname = mainwin.derwin(1,5,2,45)
    n_regions = read_str(winname)

    try:
        n_regions = int(n_regions)
    except:
        raise ValueError("Please enter an integer number of regions.")
    
    sim = Simulation(network = net,controller=controller,simparams=simparams,mode = 'Regions',gui = 'False',)

    ### Simulation Running ###
    mainwin.clear()
    mainwin.addstr(f"Running Simulation... \n \n")
    mainwin.refresh()
    sim.start_traci()
    (average_density,density,flow) = sim.run()
    sim.end_traci()
    mainwin.clear()
    mainwin.addstr(f"Simulation finished. \n \n")
    mainwin.refresh()

    # ### Clustering ###

    mainwin.clear()
    mainwin.addstr(f"Running Clustering... \n \n")
    mainwin.refresh()
    clustering = Snake_Clustering(network = net,average_density=average_density,n_regions=n_regions,n_workers = os.cpu_count(),
                                  snake_length = int(net.get_n_edges()/3), type_of_clustering='SpectralClustering')
    clustering.compute_snakes()
    clustering.similarity_matrix()
    clustering.clustering()
    labels = clustering.get_labels()
    variance = clustering.get_variance()
    simatrix = clustering.get_similarity_matrix()
    snakes = clustering.get_snakes()
    mainwin.clear()
    mainwin.addstr(f"Clustering Successful... \n \n")
    mainwin.refresh()

    # ### Saving ###

    save(f"./dep/sumo_files/{name}/regions/labels.npy",labels)
    save(f"./dep/sumo_files/{name}/regions/variance.npy",variance)
    save(f"./dep/sumo_files/{name}/regions/simatrix.npy",simatrix)
    save(f"./dep/sumo_files/{name}/regions/snakes.npy",snakes)          # These are discts, how to save them?