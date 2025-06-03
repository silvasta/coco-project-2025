from numpy import array,zeros,mean,polynomial
from scipy.optimize import minimize_scalar
from copy import deepcopy

class Region():

    def __init__(self, id, edges):
        self.id = id
        self.edges = edges
        self.edge_ids = [edge.get_id() for edge in edges]
        self.init_perimeter()
        self.mfd = None
        self.pwa = None
    

    def get_state(self,cols = []):
        '''Returns the state of the region'''
        return array([edge.get_state(cols) for edge in self.edges]).mean(axis = 0)

    def init_perimeter(self):
        
        perimeter = set()
        for edge in self.edges:
            for neigh in edge.neighbours:
                if neigh not in self.edge_ids:
                    perimeter.add(neigh)
        self.perimeter = perimeter
        

    def approximate_MFD(self,density,flow, degree = 2):
        '''Produces a polynomial approximation of the MFD from the data
        
            Args:
                density: the density data grouped by region
                flow: the flow data grouped by region
                degree: the degree of the polynomial used in the approximation
        '''    
        self.degree = degree        
        coefficients= polynomial.polynomial.polyfit(density,flow,self.degree)
        self.mfd =  polynomial.Polynomial(coefficients)
    
    def recover_critical(self):
        '''Get the critical (n_crit) and the maximum (n_max) accumulation based on the fitted MFD.'''
        r = self.mfd.roots()
        real_valued = r.real[abs(r.imag)<1e-5]
        self.n_max = real_valued[1]
        self.n_crit = minimize_scalar(lambda x: -self.mfd(x), bounds=[0, self.n_max],method='bounded').x
        
    
    def approximate_PWA(self, n_PWA):
        '''Generate the piecewise approximation (PWA) of the MFD.

        Args:
            n_PWA: Number of piecewise affine functions to use. Should be an even number.
        '''
        # Find points to use as nodes. We want one node as the critical accumulation
        # Number of nodes = n_PWA + 1
        self.n_PWA = n_PWA
        n_interpol = int(self.n_PWA / 2)
        nodes = []
        nodes.extend((0, self.n_crit, self.n_max))

        for i in range(n_interpol - 1):
            nodes.append((i + 1) * self.n_crit / n_interpol)
            nodes.append(self.n_crit + (i + 1) * (self.n_max - self.n_crit) / n_interpol)
            
        nodes.sort()
        self.pwa = []
        
        for left, right in zip(nodes, nodes[1:]):
            f_lin = polynomial.Polynomial.fit((left, right), (self.mfd(left), 
                                                self.mfd(right)),1, window=[left, right])
            self.pwa.append(f_lin)

    def get_edges(self):
        return self.edges

    def get_num_edges(self):
        return len(self.edges)
    
    def get_mfd(self):
        return self.mfd
    
    def get_pwa(self): 
        return self.pwa

    def get_pstar(self):
        return self.n_crit
    
    def get_pmax(self):
        return self.n_max

    def get_flowstar(self):
        return self.mfd(self.n_crit)

    
