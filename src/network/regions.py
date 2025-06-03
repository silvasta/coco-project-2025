from collections import Counter
from multiprocessing.pool import ThreadPool
from src.network.region import Region

from numpy import array,hstack,zeros,tile,load,eye

class Regions():

    def __init__(self,labels,edges,edges_encoding, n_PWA) -> None:
        self.labels = labels 
        self.edges = edges
        self.n_edges = len(edges)
        self.edges_encoding = edges_encoding
        self.regions = []
        self.n_regions = labels.max() + 1
        self.init_regions(self.n_regions,labels)
        self.onehotencoding()
        self.n_PWA = n_PWA
        
    
    def init_regions(self,n_regions = int,labels = array):
        '''Initializes regions
        
            Args: 
                n_regions  (int): the number of regions
                labels (np.array): the list of labels associated to the edges
        '''
        self.labels = labels
        self.n_regions = n_regions
        self.encoding = {}
        byregion = {i: [] for i in range(n_regions)}
        for edge in self.edges:
            label = labels[self.edges_encoding[edge.get_id()]]
            edge.init_region(label)
            byregion[label].append(edge)

        self.regions = [Region(i,byregion[i]) for i in range(n_regions)]
        self.perimeter = list(set().union(*[region.perimeter for region in self.regions]))
        self.init_adjacency()
        
    
    def onehotencoding(self):
        '''One hot encoding of the labels'''
        self.onehot = eye(self.n_edges,self.n_regions)
        for i in range(self.n_edges):
            for j in  range(self.n_regions):
                if self.edges[i].get_region() == j:
                    self.onehot[i,j] = 1

    def group_by_region(self,matrix):
        '''Groups edgewise matrix data into regionwise

            Args:
                matrix: The data matrix to be grouped
            Returns:
                regional_matrix: The grouped data, regions are rows
        '''
        n_edges = array([len(region.edges) for region in self.regions])
        regional_matrix = (matrix@self.onehot)/n_edges
        
        return regional_matrix

    def get_state(self,cols = []):
        '''Returns the state of the regions'''
        
        with ThreadPool(self.n_regions) as pool:
            results = array(pool.map(lambda x: x.get_state(cols),self.regions))

        return results
    
    def approximate_MFD(self,mfddata,degree,n_PWA = 10):
        '''Approximates the MFD of the regions'''
        self.mfddata = load(mfddata) if isinstance(mfddata,str) else mfddata
        self.degree = degree
        
        for i in range(self.n_regions):
            self.regions[i].approximate_MFD(self.mfddata[i,0,:],self.mfddata[i,1,:],degree = self.degree)
            self.regions[i].recover_critical()
            self.regions[i].approximate_PWA(n_PWA)
        
    def get_r(self,prediction_horizon, mode = 'density'):
        '''get method for the trajectory composed by the regional n_crit points used by DeePC
        
            prediction_horizon: the prediction horizon N of DeePC
            mode: mode of DeePC used    
        '''
        
        if mode == ['density']:
            n_crit = array([region.n_crit for region in self.regions])
            self.r = tile(n_crit,prediction_horizon)
            
        elif mode== ['flow']:
            max_flows =array([region.mfd(region.n_crit) for region in self.regions])
            self.r = tile(max_flows,prediction_horizon)
        
        elif mode == ['density','flow']:
            n_crit = array([region.n_crit for region in self.regions])
            max_flows = array([region.mfd(region.n_crit) for region in self.regions])
            r = hstack([n_crit, max_flows])
            self.r = tile(r,prediction_horizon)
        
        return self.r
    
    def get_mfd(self):
        return {"mfd": [region.get_mfd() for region in self.regions], "pwa": [region.get_pwa() for region in self.regions]}
    
    def get_perimeter(self):
        '''Returns the perimeter of the regions'''
        return self.perimeter

    def get_num_edges_for_all_regions(self):
        '''
        Returns list of 
        '''
        num_edges_list_by_region = []
        for region in self.regions:
            num_edges_list_by_region += [region.get_num_edges()]
        return num_edges_list_by_region

    def init_adjacency(self):
        'Uses neighbours of regions to construct the adjacency matrix'
        adjacency = zeros([self.n_regions,self.n_regions])
        for region in self.regions:
            for edgeid in region.perimeter:
                adjacency[region.id,self.edges[self.edges_encoding[edgeid]].get_region()] = 1
        self.adjacency = adjacency
    
    def get_adjacency(self):
        return self.adjacency
    
    def clean_regions_auto(self):
        '''The post-processing automatic cleaning procedure for the labels'''
        
        for edge in self.edges:
            neighbouring_regions = [self.edges[neigh].get_region() for neigh in self.edges[edge].get_neighbours()]        
            counts = Counter(neighbouring_regions)
    
            if neighbouring_regions.count(self.edges[edge].get_region()) <= 2:               
                self.edges[edge].init_region(counts.most_common(1)[0][0])
                               
    def clean_regions_manually(self,new_regions ={} ):
        '''Utility to modify manually labels'''
        
        for edge in new_regions:
            self.edges[self.edges_encoding[edge]].init_region(new_regions[edge])
