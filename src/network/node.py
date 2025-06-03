
class Node():
    '''
    This class represents a node (intersection) in a traffic network.
    It uses methods from the SUMO traffic simulation package.
    
    Attributes:
        sumo_obj: The SUMO object representing the node
        id: The ID of the node
        incoming: The IDs of all incoming edges
        outgoing: The IDs of all outgoing edges
        type: The type of node: priority, traffic light, etc.
        coordinates: The coordinates of the node
        node_neighbours: The neighboring nodes
        edge_neighbours: The neighboring edges
    '''
    
    def __init__(self,sumo_obj) -> None:
        '''
        Constructor for the Node class. Initializes attributes using the provided SUMO object.
        '''
        self.sumo_obj= sumo_obj  # The SUMO object representing the node
        self.id = sumo_obj.getID()  # The ID of the node
        self.incoming = [edge.getID() for edge in sumo_obj.getIncoming()]  # The IDs of all incoming edges
        self.outgoing = [edge.getID() for edge in sumo_obj.getOutgoing()]  # The IDs of all outgoing edges
        self.type = sumo_obj.getType()  # The type of the node
        self.coordinates = sumo_obj.getCoord()  # The coordinates of the node
        self.node_neighbours = sumo_obj.getNeighboringNodes(outgoingNodes=True, incomingNodes=True)  # The neighboring nodes
        self.edge_neighbours = self.incoming+ self.outgoing  # The neighboring edges
    
    def is_perimeter(self,i = int):
        '''
        Sets the perimeter attribute, as a 0 or 1 representing a bool.
        '''
        self.perimeter = i
            
    # The following methods are getters for the corresponding attributes
    def get_object(self):
        return self.sumo_obj

    def get_id(self):
        return self.id
    
    def get_incoming(self):
       return self.incoming

    def get_outgoing(self):
        return self.outgoing
    
    def get_coordinates(self):
        return self.coordinates
    
    def get_node_neighbours(self):
        return self.node_neighbours
    
    def get_edge_neighbours(self):
        return self.edge_neighbours

    def get_type(self):
        return self.type
    
    def get_perimeter(self):
        return self.perimeter