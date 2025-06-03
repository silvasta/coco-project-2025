import os,sys,json
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.clustering.clusteringalgorithm import ClusteringAlgorithm
from src.network.network import Network

class ManualSelection(ClusteringAlgorithm):
    def __init__(self, regions, network=Network) -> None:
        self.edges = network.get_edges()
        self.n_edges = network.get_n_edges()
        with open(regions, 'r') as file:
            self.regions = json.load(file)

    def clustering(self):
        
        edge_to_region = {edge: int(region) for region, edges in self.regions.items() if 'target' not in region for edge in edges}
        labels = [edge_to_region.get(edge.sumoid, -1) for edge in self.edges]

        return labels
        