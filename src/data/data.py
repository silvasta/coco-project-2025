from dataclasses import dataclass
from numpy import array,save

@dataclass
class Data():

    id = str
    data: array 

    def save(self,path):
        save(path,self.data)