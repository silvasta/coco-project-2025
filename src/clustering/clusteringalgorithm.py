from abc import ABC, abstractmethod

class ClusteringAlgorithm(ABC):

    def __init__(self, data):
        self.data = data

    @abstractmethod
    def clustering(self):
        pass