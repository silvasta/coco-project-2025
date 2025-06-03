from lxml import etree
from numpy import zeros,save
from copy import deepcopy

class Theta():

    def __init__(self,tripinfopath,network):
        self.network = network
        self.build_theta(tripinfopath,network)
    
    def get_theta(self):
        return self.T
    
    def build_theta(self,xmlfile,network):
        """- routing : fraction of the people theta_ij^h that want to go from i to j passing through h"""

        parser = etree.XMLParser(recover=True)
        tree = etree.parse(xmlfile,parser= parser)
        root = tree.getroot()
        routes = {}
        total_veh = len(root)
        n_regions = network.get_n_regions()
        Theta = zeros((n_regions,n_regions,n_regions))
        
        counter = 0
        for veh in root:
            
            id = veh.values()[0]
            try:
                routes[id]= veh.__getitem__(0).values()[0].split(' ')
            except:
                counter += counter
                print('number of routes skipped: ',counter)
                pass
            
        i ={}
        j ={}
        passing_regions ={}   
        edges = network.get_edges()
        encoding = network.get_edges_encoding()

        for id in routes:
            route = routes[id]
            if len(route) == 1:
                i[id] = edges[encoding[route[0]]].get_region()
                j[id] = i[id]
                passing_regions[id] = set()
            else:
                i[id] = edges[encoding[route[0]]].get_region()

                if route[len(route)-1] == '':
                    route.remove('')

                j[id] = edges[encoding[route[len(route)-1]]].get_region()
                route.remove(route[0])
                route.remove(route[len(route)-1])
                passing_regions[id] = []
                
                for edge in route:
                    
                    passing_regions[id].append(edges[encoding[edge]].get_region())
                    
                passing_regions[id] = set(passing_regions[id])
                if i[id] in passing_regions[id]:
                    passing_regions[id].remove(i[id])
                if j[id] in passing_regions[id]:
                    passing_regions[id].remove(j[id])
        
                
        for id in routes:
            if len(passing_regions[id]) == 0:
                Theta[i[id],j[id],j[id]] +=1

            if len(passing_regions[id]) ==1:
                h = list(passing_regions[id])[0]
                Theta[i[id],j[id],h] +=1

            if len(passing_regions[id])>1:
                hs = list(passing_regions[id])
                div =len(hs)
                
                for h in hs:
                    Theta[i[id],j[id],h]+=(1/div)
                  
        Theta = Theta/total_veh
        self.T = Theta        
        

        