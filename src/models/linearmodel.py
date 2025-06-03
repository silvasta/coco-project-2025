import numpy as np 
import networkx as nx

class LinearModel():
    '''Implements the linear aggregated model of a traffic network.
        Source: https://www.sciencedirect.com/science/article/pii/S2405896317319572

    Attributes:
        network (Network): An instance of the Network class representing the traffic network.
        MFDs (list): A list of Macroscopic Fundamental Diagrams for each region in the network.
        PWA (list): A list of Piecewise Affine approximations of the MFDs.
        n_regions (int): The number of regions in the network.
        graph (Graph): A networkx graph representing the traffic network.
        Theta (numpy.ndarray): The routing 3d tensor of cars in the network.
        A (numpy.ndarray): A matrix representing the state dynamics of the system.
        B (numpy.ndarray): A matrix representing the input weights.'''
    
    def __init__(self, network = None, demand = None):
        '''
        Initializes the LinearModel class with given parameters and constructs the matrices A and B.
        '''
        
        self.network = network
        out = network.get_mfd()
        self.MFDs = out['mfd']
        self.PWA = out['pwa']
        self.n_regions = network.n_regions
        self.Regions = network.regions
        self.graph = network.graph
        # self.Theta = network.Theta

        # gamma converts density to accumulation
        # accumulation = density * gamma
        self.gamma = np.zeros(self.n_regions)
        for i in range(self.n_regions):
            num_edges = self.network.regions.regions[i].get_num_edges()
            lanes_per_edge = self.network.get_const_lane_number()
            edge_length = self.network.get_const_edge_length_km()
            self.gamma[i] = num_edges * lanes_per_edge * edge_length
    
        # Construct empty linear system
        # User needs to explicitly call linearize around (Ts, rho_star, v_star)!
        self.A = np.eye(self.n_regions)
        self.B = np.zeros((self.n_regions, self.n_regions - 1))
        self.C = np.eye(self.n_regions)
        self.d = np.zeros(self.n_regions)


    def get_pairwise_demand(self, T, Ts):
        """
        Returns the total demand between each pair of regions over a given time period.

        Args:
            T (int): The start time.
            Ts (int): The time period length.
        """
        
        q = self.network.demand
        demand = np.zeros((self.n_regions, self.n_regions))
        
        for i in range(self.n_regions):
            for j in range(self.n_regions):
                demand[i, j] = q[i][j][T:T+Ts].sum()
        
        return demand
     
    def get_origin_demand(self, T, Ts):
        """
        Returns a demand vector q, which represents the total number of cars over a given time period 
        **originating from each region i.**
        """
        q = np.zeros((n, Np))
        # q = demand.T
        for k in range(Np):
            Q_full = self.get_demand(T + k * Ts, Ts)
            q[:, k] = Q_full.sum(axis=1)

        return q

    # def linearize(self, n_regions, Ts, rho_star, v_star):
    # def linearize(self, Ts, rho_star, v_star):
    def linearize(self, Ts, rho_star, u_star):
        n = self.n_regions # fixed to 5!!
        Thr = Ts / 3600  # convert to hours
        epsilon = (0.9 ** (1 / 20)) ** Ts  # completion rate over Ts
        # print(epsilon)
        # Compute A matrix
        A = np.eye(5)
        A[0, 0] -= 50 * Thr / self.gamma[0] * u_star[1]
        A[1, 1] -= 50 * Thr / self.gamma[1] * (u_star[2] + u_star[3])
        A[2, 2] -= 50 * Thr / self.gamma[2] * u_star[4]
        A[3, 3] -= 50 * Thr / self.gamma[3] * u_star[0]
        A[4, 0] =  50 * Thr / self.gamma[4] * u_star[1]
        A[4, 1] =  50 * Thr / self.gamma[4] * (u_star[2] + u_star[3])
        A[4, 2] =  50 * Thr / self.gamma[4] * u_star[4]
        A[4, 3] =  50 * Thr / self.gamma[4] * u_star[0]
        A[4, 4] =  epsilon

        # Compute B matrix
        B = np.zeros((5,5))
        B[0, 1] = -50 * Thr / self.gamma[0] * rho_star[0]
        B[1, 2] = -50 * Thr / self.gamma[1] * rho_star[1]
        B[1, 3] = -50 * Thr / self.gamma[1] * rho_star[1]
        B[2, 4] = -50 * Thr / self.gamma[2] * rho_star[2]
        B[3, 0] = -50 * Thr / self.gamma[3] * rho_star[3]
        B[4, 0] =  50 * Thr / self.gamma[4] * rho_star[3]
        B[4, 1] =  50 * Thr / self.gamma[4] * rho_star[0]
        B[4, 2] =  50 * Thr / self.gamma[4] * rho_star[1]
        B[4, 3] =  50 * Thr / self.gamma[4] * rho_star[1]
        B[4, 4] =  50 * Thr / self.gamma[4] * rho_star[2]

        # Matrix C (for demand q(k))
        C = np.diag(Thr / self.gamma)

        # Compute d vector
        d = np.zeros(5)
        d[0] =  50 * Thr / self.gamma[0] * u_star[1] * rho_star[0]
        d[1] =  50 * Thr / self.gamma[1] * (u_star[2] + u_star[3]) * rho_star[1]
        d[2] =  50 * Thr / self.gamma[2] * u_star[4] * rho_star[2]
        d[3] =  50 * Thr / self.gamma[3] * u_star[0] * rho_star[3]
        d[4] = -50 * Thr / self.gamma[4] * (
            u_star[1] * rho_star[0] +
            (u_star[2] + u_star[3]) * rho_star[1] +
            u_star[4] * rho_star[2] +
            u_star[0] * rho_star[3]
        )

        self.A, self.B, self.C, self.d = A.copy(), B.copy(), C.copy(), d.copy()
        return A.copy(), B.copy(), C.copy(), d.copy()

