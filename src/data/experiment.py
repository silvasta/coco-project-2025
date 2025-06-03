import pandas as pd
import os,json,zipfile
import time
from tools.utils import zipdir
#from matplotlib import rc
import matplotlib.pyplot as plt
# rc('font',**{'family':'serif','serif':['Roman']})
# rc('text', usetex=True)
# rc('text.latex', preamble=r'\usepackage{amsmath}')

class Experiment():
    '''This class acts as a tracker for all the files used when running a simulation as well as the results obtained.
        It is used to save the results in the out folder, and to load them for further analysis.
        
        Attributes:
            - id: unique identifier for the experiment, used as id in database
            - name: name of the network
            - description: brief description of the experiment
            - info: dictionary containing the paths to the files used in the simulation
            - results: dictionary containing the results of the simulation
            - output_path: output path were to save the results
    '''

    def __init__(self,info = None,description = ''):
        self.date = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.description = description
        self.info = info if info is not None else None
        self.id = self.info['taskparams']['id'] if info is not None else None
        self.results = info['results'] if info is not None else None
        self.output_path = info['output_path'] if info is not None else None

    def save(self):
        '''Saves an experiment to the out folder'''

        os.makedirs(self.output_path+'results/')    
        [self.results[name].to_csv(self.output_path+'results/'+name+'.csv') for name in self.results]
        
        with open(self.output_path+'info.json','w') as f:
            json.dump(self.info['taskparams'],f)

        print(f"Saving experiment results to {self.output_path}. This directory can be copied and renamed if you want to retain the results, otherwise it will be overwritten during the next experiment!")
                
    def load(self,path = str):
        '''Loads an experiment from the out folder'''
        self.info = json.load(open(path+"/info.json",'r'))
        self.id = self.info['id']
        res = path+"/results/"
        files = os.listdir(res)
        self.results = {f"{file[:-4]}":pd.read_csv(res+file,index_col= 0) for file in files }
        
    
    def pack(self):
        '''Packs an experiment into a .zip file'''
        with zipfile.ZipFile("name.zip","w",zipfile.ZIP_DEFLATED) as zipf:
            zipdir(self.output_path,zipf)
    
    def summary(self):
        '''Displays the summary of an experiment'''
        print(f"Experiment on {self.info['taskparams']['networkname']} with id {self.id} \n")
        print(f"Controller used :{self.info['taskparams']['controller']}\n \n")
        print("-"*30+"\n")
        print("Description: \n")
        print(self.description)
        print("-"*30+"\n \n")
        print("Summary Metrics: \n")
        print(self.results['metrics'])

    def plot(self):
        '''Plots the results of an experiment'''
        pass 