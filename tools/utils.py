from itertools import groupby
import sys,os,ast,pickle,importlib,re,json

import matplotlib.pyplot  as plt
import numpy as np
from lxml import etree
from pandas import DataFrame

def alpha_to_gamma(alpha, theta):
    '''Derive the flow matrix a given alpha and routing theta

    Args:
        alpha: Matrix enconding the destination of the cars in a region
        theta: The routing tensor

    Returns:
        A flow matrix, namely to which neighboring region the cars in a given
        region travel to.
    '''
    n_regions = alpha.shape[0]
    gamma = np.zeros((n_regions, n_regions))
    for i in range(n_regions):
        for j in range(n_regions):
            gamma[i, j] = alpha[i, :] @ theta[i, :, j]

    return gamma

class pickler():
    ''''''

    def save(self,object,name):
        with open(name, "wb") as f:
            pickle.dump(object,f)
    
    def load(self,name):
        with open(name, "rb") as f:
             obj = pickle.load(f)
        
        return obj

def zipdir(path, ziph):
    # ziph is zipfile handle
    for root, dirs, files in os.walk(path):
        for file in files:
            ziph.write(os.path.join(root, file), 
                       os.path.relpath(os.path.join(root, file), 
                                       os.path.join(path, '..')))

'''    Function to check weather all the elements in an array are equal  '''

def all_equal(iterable):
    g = groupby(iterable)
    return next(g, True) and not next(g, False)

''' Function to Preprocess the names of the nodes in traffic lights clusters '''

def preprocessing(tls):
    ''' Preprocessing for the tls ids in Zurich network
    '''
    nodes_of_tls = {tl.getID():[] for tl in tls}
    
    for id in nodes_of_tls:
        if 'GS_' in id:
            nodes_of_tls[id].append(id[3:])

        elif id.isnumeric():
            nodes_of_tls[id].append(id)

        elif 'cluster' in id:
            if 'joinedS' in id:
                indexes = [m.start() for m in re.finditer('cluster', id)]
                first_nodes = id[8:indexes[0]].split('_')

                for node in first_nodes:
                    if node != '':
                        nodes_of_tls[id].append(node)
                i = 0

                while i < len(indexes)-1:
                    nodes_of_tls[id].append(id[indexes[i]:indexes[i+1]-1])
                    i+=1
                    
                nodes_of_tls[id].append(id[indexes[i]:]) 
            else: 
                nodes_of_tls[id].append(id)
    return  nodes_of_tls 

def progressBar(count_value, total, suffix=''):
    '''Simple Progress Bar utility'''
    bar_length = 100
    filled_up_Length = int(round(bar_length* count_value / float(total)))
    percentage = round(100.0 * count_value/float(total),1)
    bar = '=' * filled_up_Length + '-' * (bar_length - filled_up_Length)
    sys.stdout.write('[%s] %s%s ...%s\r' %(bar, percentage, '%', suffix))
    sys.stdout.flush()
    
def write_taz(objects,path,byregion = False):

    ### TO BE ADAPTED TO NEW FRAMEWORK ###
    if byregion:
        color = ["#05f746","#f70505","#0509f7","#f705d3","#f7e305","#080700","#f78502","#02f7f3","#b602f7"]
        with open(f"regions.taz.xml","w") as taz:
            
            taz.write('<tazs>')
            
            for region in objects:
                
                taz.write('    <taz id="%s" ' % (region))
                taz.write ('edges="')
                for edge in objects:
                    taz.write(f'{edge} ')
                taz.write('" color= "%s" />\n' % (color[region]))

            taz.write('</tazs>\n')
        
        print('Writing TAZ : Success')
    else: 

        color = "#de1709"
        with open(path,"w") as taz:
            taz.write('<tazs>\n')
            taz.write('    <taz id="0" ')
            taz.write ('edges="')
            for edge in objects.index:
                taz.write(f'{edge} ')
            taz.write('" color= "%s" />\n' % (color))
            taz.write('</tazs>')
        print('Writing TAZ : Success')


def compute_demand(xmlfile,duration,byregion = False,regions = '',grid = False, window = 60):
    ''' Computes the demand from a given xml file'''
    parser = etree.XMLParser(recover=True)
    tree = etree.parse(xmlfile,parser= parser)
    root = tree.getroot() 

    if byregion:
        with open(regions,'r') as f:
            regions = json.load(f)
        for reg in regions:
            regions[reg] = set(regions[reg])
        nregions = len(regions)
        total = np.zeros((duration,nregions,nregions))
        cols = [f'from {i} to {j}' for i in regions.keys() for j in regions.keys()]


        for veh in root:
            t = int(float(veh.values()[1]))     
            route = veh.getchildren()[0].values()[0].split(' ')
            dep_edge = route[0]
            arr_edge = route[-1]
            # dep_edge = veh.values()[2]
            # arr_edge = veh.values()[3]
            departure_region =[int(reg) for reg in regions if dep_edge in regions[reg]]
            arrival_region = [int(reg) for reg in regions if arr_edge in regions[reg]]
            #total[t-54000,departure_region,arrival_region]+=1
            total[t,departure_region,arrival_region]+=1

        total = total.reshape(duration,nregions**2)

        demand = DataFrame(total,columns = cols)
    else:  
        cols = ['total demand']      
        total = np.zeros(duration)
        for veh in root:
            t = int(float(veh.values()[1]))    
            total[t-54000]+=1
            
        demand = DataFrame(total,columns = cols)
    demand = demand.rolling(window=600).mean().dropna()
    demand = demand.groupby(demand.index//window).sum()
    
  
    return demand


def depart_arrival(xmlfile,):
    
    parser = etree.XMLParser(recover=True)
    tree = etree.parse(xmlfile,parser= parser)
    root = tree.getroot()
    n_veh = len(root)
    depart = np.zeros(n_veh)
    arrival = np.zeros(n_veh)
    i = 0

    for veh in root:    
        depart[i] = veh.values()[2]
        arrival[i] = veh.values()[8]
        i+=1
    
    return depart,arrival


def plot_single(array,ylabel, region):
    
    color = ["#05f746","#f70505","#0509f7","#f705d3","#f7e305","#080700","#f78502","#02f7f3","#b602f7"]
    
    x = np.arange(array.shape[0])
    
    plt.plot(x,array,color[region])
    plt.grid()
    plt.ylabel(ylabel)
    plt.xlabel('Time')
    plt.show()
    
    
def plot_multiple(matrix,ylabel):
    
    color = ["#05f746","#f70505","#0509f7","#f705d3","#f7e305","#080700","#f78502","#02f7f3","#b602f7"]
    
    x = np.arange(matrix.shape[1])
    
    plt.figure(figsize=(16,9))
    
    for i in range(matrix.shape[0]):
        
        plt.plot(x,matrix[i,:],color[i])
    
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel(ylabel)
    plt.show()

class Parser():
    def __init__(self,rootdir):
        self.rootdir = rootdir
        self.listfiles()
        self.parse()

    def show_info(self,functionNode):
        print("Function name:", functionNode.name)
        print("Args:")
        for arg in functionNode.args.args:
            #import pdb; pdb.set_trace()
            print("\tParameter name:", arg.arg)
    
    def listfiles(self):
        self.paths = []
        for root, dirs, files in os.walk(self.rootdir):
            for file in files:
                if file.endswith('.py'):
                    self.paths.append(os.path.join(root,file))

        self.paths = [path.replace('\\','/') for path in self.paths]

    def parse(self):
        self.functions = []
        self.classes = []
        self.modules = []
        for path in self.paths:
            
            with open(path) as file:
                node = ast.parse(file.read())
                

            self.functions += [n for n in node.body if isinstance(n, ast.FunctionDef)]
            self.classes += [n for n in node.body if isinstance(n, ast.ClassDef)]
            self.modules += [path for n in node.body if isinstance(n, ast.ClassDef)]

        self.modules = [path[2:].replace('/','.')[:-3] for path in self.modules]
        self.modules = {class_.name : module for class_,module in zip(self.classes,self.modules)}
        self.classes = {i.name: i for i in self.classes}
        self.functions = {i.name: i for i in self.functions}

    def import_class(self, class_name):
        
        module = self.modules[class_name]
        class_ = getattr(importlib.import_module(module),class_name)
        return class_

    def show(self):
        for function in self.functions:
            self.show_info(function)

        for class_ in self.classes:
            print("Class name:", class_.name)
            methods = [n for n in class_.body if isinstance(n, ast.FunctionDef)]
            for method in methods:
                self.show_info(method)
        
        for path in self.paths:
            print(path)
