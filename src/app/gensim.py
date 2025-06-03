import os,sys,json
# setting the path for the imports
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from logs.error_handling.exceptions import *
from tools.generator import Generator

def gensim(mainwin,*args):
    net = args[0]
    name = args[1]
    pos =(2,0)
    arr_dep = json.load(open(f"./dep/sumo_files/{name}/network/departure_arrival.json",'r'))
    departure_set =  arr_dep["departure_set"]
    arrival_set = arr_dep["arrival_set"]

    ### insert Menus to control demand generation (better graphical interface) ###
    
    gen = Generator()
    gen.generateTrips(name,rate = 0.5,departure_set = departure_set,arrival_set = arrival_set)
    gen.generate_sumocfg(name)