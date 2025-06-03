from scipy.sparse import csr_matrix
from sklearn.cluster import SpectralClustering, AgglomerativeClustering, DBSCAN, OPTICS 
import matplotlib.pyplot as plt
from src.network.network import Network
from copy import deepcopy
from operator import itemgetter
from multiprocessing.pool import ThreadPool
from itertools import combinations_with_replacement
import pandas as pd
import numpy as np
from src.clustering.clusteringalgorithm import ClusteringAlgorithm

'''To be written in C using Cython'''

class SnakeClustering(ClusteringAlgorithm):
    '''Implements the Snake Clustering algorithm.
        Source: https://www.sciencedirect.com/science/article/pii/S0191261515302605
        
        Attributes:
            net: The SUMO object representing the network.
            edges: A dictionary containing the edges of the network.
            nodes: A dictionary containing the nodes of the network.
            n_edges: The number of edges in the network.
            n_workers: The number of workers to use for parallelization.
            average_density: A dictionary containing the average density of each edge.
            ids: A list containing the ids of the edges.
            type: The type of clustering to use.
            n_regions: The number of regions to find.
            counter_snakes: A counter for the number of snakes.
            counter_matrix: A counter for the number of matrix computations.
            snake_length: The length of the snakes.
    '''
    def __init__(self,network = Network ,average_density = {},n_regions = int,n_workers = 1 ,snake_length = int,type_of_clustering = 'SpectralClustering') -> None:
        """Constructor for the Snake_Clustering class."""
        
        # Network related attributes
        self.net = network.get_object()
        self.edges = network.get_edges()
        self.nodes = network.get_nodes() 
        self.n_edges = network.get_n_edges()
        
        # Other attributes
        self.n_workers = n_workers
        self.average_density = average_density
        self.ids = list(self.edges.keys())
        self.type = type_of_clustering
        self.n_regions = n_regions
        self.counter_snakes = 0
        self.counter_matrix = 0
        
        # Set snake_length
        if snake_length == int:
            self.snake_length = self.n_edges-1
        else:
            self.snake_length = snake_length
 
    def compute_snakes(self):
        """Computes the "snakes", which are paths in the network. 
        
        Uses a ThreadPool to parallelize the computation.
        """
        # Initialization
        self.variance = {edge: [] for edge in self.edges}
        snakes = {edge: [edge] for edge in self.edges}
        snake_neighbours = {edge: self.edges[edge].get_neighbours() for edge in self.edges}  
        tuples = [(snake_id,snakes[snake_id],snake_neighbours[snake_id])for snake_id in snakes]
        
        # Start computation
        with ThreadPool(self.n_workers) as pool1:
            results = pool1.starmap(self.compute_snake,tuples)
            
            # Update snakes with results
            for i_tuple, result in zip(tuples,results):
                snake_id,snake, snake_n = i_tuple
                snakes[snake_id]= result
        
        # Update class attributes
        self.variance = pd.DataFrame(self.variance)
        self.snakes = snakes
    
    def compute_snake(self,snake_id,snake,snake_neighbours):
        """Computes a single snake. 
        
            Finds the minimum distance (in terms of variance) edge and adds it to the snake.
        
            Requires:
                snake_id: The id of the snake.
                snake: The snake initialized as a list containing only the snake_id.
                snake_neighbours: The neighbours of the snake.
            
            Returns:
                snake: The snake.
        """
        
        # Initialization
        edges_left = deepcopy(self.ids) 
        edges_left.remove(snake_id)
             
        # Compute snake
        while len(edges_left)> self.n_edges - self.snake_length:
            best = self.find_minimum(snake, snake_neighbours)
            snake.append(best)
            snake_neighbours.update(self.edges[best].get_neighbours())
            snake_neighbours = snake_neighbours - set(snake)  
            edges_left.remove(best)
        
        # Finalize snake
        if self.snake_length == self.n_edges-1:
            snake.append(list(snake_neighbours)[0])
        
        # Update counter
        self.counter_snakes +=1
        
        return snake
           
    def find_minimum(self,snake,snake_neighbours):  
        """Finds the edge with the minimum distance (in terms of variance) to the current snake.
        
            Requires:
                snake: The current snake.
                snake_neighbours: The neighbours of the snake.
            
            Returns:
                best_index: The index of the edge with the minimum distance (in terms of variance) to the snake.
        """
        
        # Compute distances
        snake_neighbours = list(snake_neighbours)
        densities_snake = itemgetter(*snake)(self.average_density)
        densities_neighbours = itemgetter(*snake_neighbours)(self.average_density)
        self.variance[snake[0]].append(np.var(densities_snake))
        average_snake =np.average(densities_snake)
        distances = np.abs(average_snake-densities_neighbours)
        
        # Find minimum
        if type(distances) == list:    
            best_index = np.where(distances == min(distances))[0][0]
        else: 
            best_index = 0

        return snake_neighbours[best_index]
    
    def similarity_matrix(self):
        """Computes a similarity matrix for the snakes. 
        
        Uses a ThreadPool to parallelize the computation.
        """
        
        # Initialization
        W = np.zeros((self.n_edges,self.n_edges)) #make this a sparse matrix    
        snake_length = len(self.snakes[self.ids[0]])
        step = int(snake_length/10)
        indexes = list(combinations_with_replacement(range(self.n_edges),2))
        indexes = [(i_a,i_b,snake_length,step) for i_a,i_b in indexes]
        
        # Start computation
        with ThreadPool(self.n_workers) as pool2:
            results  = pool2.starmap(self.compute_intersection,indexes)

            # Update W with results
            for i_tuple, result in zip(indexes,results):                
                i,j,t,step = i_tuple
                W[i,j] = W[j,i] = result
        
        # Update class attribute
        self.W = csr_matrix(W/W.max())
    
    def compute_intersection(self,i,j,snake_length,step):
        """Computes the intersection of two snakes.

            Requires:
                i: The index of the first snake.
                j: The index of the second snake.
                snake_length: The length of the snakes.
                step: The step size for the intersection computation.
            
            Returns:
                The cardinality of the intersection between the two snakes as a int.
        """
        
        # Update counter
        self.counter_matrix +=2
        
        return np.sum([len(set(self.snakes[self.ids[i]][:k]).intersection(set(self.snakes[self.ids[j]][:k]))) for k in np.arange(1,snake_length,step)])

    def set_W(self,W):
        """ Sets the similarity matrix W, if already computed.

            Requires:
                W: The similarity matrix.
        """
        self.W = W
    
    def clustering(self):
        """Performs the actual clustering on the similarity matrix W. 
        
            Supports several types of clustering.
        """
                
        if self.type == 'SpectralClustering':
            SPC = SpectralClustering(n_clusters= self.n_regions,affinity='precomputed')
            self.labels = SPC.fit_predict(self.W)
            
        if self.type == 'AgglomerativeClustering':
            SPC = AgglomerativeClustering(n_clusters= self.n_regions,metric='precomputed',linkage='single')
            self.labels = SPC.fit_predict(self.W)
        
        if self.type == 'OPTICS':
            SPC = OPTICS(min_samples=5 ,metric='precomputed',cluster_method='xi',algorithm='auto')
            self.labels = SPC.fit_predict(self.W)
        
        if self.type == 'DBSCAN':
            SPC = DBSCAN(min_samples=2 ,metric='precomputed',algorithm='auto')
            self.labels = SPC.fit_predict(self.W)
    
    def plot(self):
        """
        Plots the variance of the snakes.
        """
        self.variance.plot()
        plt.show()
    
    # Several getters for the class attributes
    def get_labels(self):
        return self.labels
        
    def get_snakes(self):
        return self.snakes
        
    def get_similarity_matrix(self):
        return self.W
    
    def get_variance(self):
        return self.variance
