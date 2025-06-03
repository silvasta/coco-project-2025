# %%
''' IMPORTS '''
import os, sys
# setting the path for the imports
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)        
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")
    
import sumolib
import traci

from numpy import array,array_split,load
from multiprocessing.pool import ThreadPool
from networkx import Graph

from network.node import Node
from actuators.edge import Edge
from network.regions import Regions
from network.region import Region

class Network():
    '''Model a city in the form of a graph, i.e. a tuple composed by a set of edges and a set of nodes.                          
                                                                                
        This class is a high level wrapper for the network object in sumolib, it is composed by edges and nodes 
        which can be traffic lights or not. The class allows to retrieve data from the simulation such as density, flow  

        Attributes:
            net_path : path of the .net.xml file    str().
            control_cycle : contains the control cycle for control   dict().
            sumo_obj : the underlying Sumolib object    sumolib object.
            n_edges : number of edges in the network    int().
            n_nodes : number of nodes in the network    int().
            n_tls : number of traffic lights            int().
            nodes : dictionary of pairs id-sumolib obj  dict().
            nodes_encoding : mapping between id-int     dict().
            edges : dictionary of pairs id-sumolib obj  dict().
            edges_encoding : mapping between id-int     dict().
            redlights : dictionary of pairs id-sumolib obj  dict().
            redlights_encoding : mapping between id-int     dict().
            control_redlights : mapping id-sumolib obj of the actuators dict().
            m : number of controlled redlights (actuators)  int().
            labels : list of labels of the edges (0:n_regions-1)    list().
            n_regions : number of homogeneous regions in the network    int().
            regions : defines the regions contains : edges, encoding, MFD, PWA  dict().
            n_crit : critical density point of the MFD.
            n_max : maximum density point of the MFD.
            
    '''

    def __init__(self,name = str(),freq = 1,n_threads = 1,labels = None, demand_path=None, theta_path=None, n_PWA=None) -> None:
        '''Initializes the instance
        
        Args :
             name: name of the network, it corresponds to the name of the folder in /dep/sumo_files;
        '''
        
        self.name = name
        self.n_threads = n_threads
        self.freq = freq
        self.labels = load(labels) if labels is not None else None
        if demand_path is not None:
            self.demand = load(demand_path)
        if theta_path is not None:
            self.Theta = load(theta_path, allow_pickle=True)
        try:
            self.sumo_obj = sumolib.net.readNet(f"./dep/sumo_files/{self.name}/network/{self.name}.net.xml")
        except:
            raise ValueError("The network file doesn't exist")
                    
        self.init_nodes()
        self.init_edges() 
        self.tls = self.sumo_obj.getTrafficLights()
        self.n_regions = self.labels.max() + 1 if labels is not None else 0 
        self.regions = Regions(self.labels,self.edges,self.edges_encoding, n_PWA) if labels is not None else None   
        self.perimeter = self.init_perimeter() if self.regions is not None else None 
        self.graph = Graph(self.regions.adjacency) if self.regions is not None else None
        self.n_edges = len(self.edges)
        self.n_nodes = len(self.nodes)
        self.n_tls = len(self.tls)
          
    def init_nodes(self):
        '''Initializes the nodes of the network'''
        
        nodes = self.sumo_obj.getNodes()
        self.nodes = array([Node(node) for node in nodes])
        self.nodes_encoding = {node.get_id(): i for i,node in enumerate(self.nodes)}

    
    def init_edges(self):
        '''Initializes the edges of the network.'''
        edges = self.sumo_obj.getEdges()
        self.edges = array([Edge(edge,self.freq) for edge in edges])
        self.edges_encoding = {edge.get_id(): i for i,edge in enumerate(self.edges)}
        self.edge_length_km = self.edges[0].get_length_km() 
        # TODO: Generalize this later, for now we just use the first edge
        self.lanes_per_edge = self.edges[0].get_lane_number()

    
    def get_state(self, byregion = False,cols = []):
        '''Returns the state of the network'''
        if byregion:
            return self.regions.get_state(cols)
        else:
            splits = array_split(self.edges, self.n_threads)
            with ThreadPool(self.n_threads) as pool:
                results  = pool.map(lambda x: x.get_state(cols),self.edges)
            state = results

            return state
    
    def init_perimeter(self):
        'Returns the nodes on the perimeter between regions'
        self.perimeter = self.regions.get_perimeter()
            

    def get_object(self):
        return self.sumo_obj

    def get_nodes(self):
         return self.nodes
    
    def get_const_edge_length_km(self):
        return self.edge_length_km

    def get_const_lane_number(self):
        return self.lanes_per_edge

    def get_edges(self):
         return self.edges
    
    def get_tls(self):
         return self.tls
    
    def get_nodes_encoding(self):
         return self.nodes_encoding
    
    def get_edges_encoding(self):
         return self.edges_encoding
     
    def get_n_edges(self):
         return self.n_edges
     
    def get_n_nodes(self):
         return self.n_nodes
    
    def get_regions(self):
        return self.regions
    
    def get_n_regions(self):
        return self.n_regions
     
    def get_n_tls(self):
         return  self.n_tls
    
    def get_mfd(self):
        return self.regions.get_mfd()
    
    def get_perimeter(self):
        return self.perimeter
    
    def get_demand(self):
        return self.demand
    
    def get_n_max(self):
        n_max = array([region.n_max for region in self.regions.regions])
        return n_max
    
    def get_n_crit(self):
        n_crit = array([region.n_crit for region in self.regions.regions])
        return n_crit
    
    def get_n_PWA(self):
        n_PWA = self.regions.n_PWA
        return n_PWA
    
    






