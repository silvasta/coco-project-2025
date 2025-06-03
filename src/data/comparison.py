import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from numpy import zeros, arange

def moving_average(array, step):

    size = array.shape
    new_array = zeros((size[0],int(size[1]/step)))
    for i in range(size[0]):
        for j in arange(0,size[1], step = step):
            new_array[i,int(j/step)] = array[i,j:j+step-1].mean() 
    return new_array

class Comparison():

    def __init__(self,experiments = list, labels = list, region = 'Region 1'):
        self.experiments = experiments
        self.region = region
        self.den = {labels[i]: exp.results['density_results'][self.region].dropna() for i,exp in enumerate(self.experiments)}
        self.den_pred = {labels[i]: exp.results['density_prediction_results'][self.region].dropna() for i,exp in enumerate(self.experiments)}
        self.inp = {labels[i]: exp.results['input_results'] for i,exp in enumerate(self.experiments)}
        self.flo = {labels[i]: exp.results['flow_results'][self.region].dropna() for i,exp in enumerate(self.experiments)}
        self.met = {'comparison': pd.concat([exp.results['metrics'] for exp in self.experiments],axis=1)}
        self.met['comparison'].columns = [labels[i] for i,exp in enumerate(self.experiments)] 

        self.figsize = (32,18)
    
    def plot(self):
        '''Plots the results of the experiments'''

        pl = [self.den ,self.flo,self.inp,self.met] 
        titles = ['Density','Flow','Input','Metrics']      
        widths = [8.0,4.0,2.0]
        fig, ax = plt.subplots(nrows = 2, ncols = 2)
        begin = self.experiments[0].info['taskparams']['begin']
        end = self.experiments[0].info['taskparams']['end']
        ticks  = np.arange(begin, end, 600)
        labels = [i/60 for i in ticks]

        for i,p in enumerate(pl):
            for key in p:
                if p is self.den:
                    p[key].plot(figsize = self.figsize,ax = ax[i//2,i%2],
                                title = titles[i],grid = True)
                    self.den_pred[key].plot(figsize = self.figsize,ax = ax[i//2,i%2], style= 'o-')
                    ax[i//2,i%2].legend([key for key in p]+['Prediction ' + key for key in p])
                    # ax[i//2,i%2].set_xticks(ticks)
                    # ax[i//2,i%2].set_xticklabels(labels)
                elif p is self.flo:
                    p[key].plot(figsize = self.figsize,ax = ax[i//2,i%2],
                                title = titles[i],grid = True)
                    ax[i//2,i%2].legend([key for key in p])
                elif p is self.inp:  
                    p[key].plot(figsize = self.figsize,ax = ax[i//2,i%2],
                                title = titles[i],grid = True) 
                    ax[i//2,i%2].legend([])
                else:
                    ax[i//2,i%2].axis('off')
                    rcolors = np.full(len(p[key].index), 'linen')
                    ccolors = np.full(len(p[key].columns), 'lavender')

                    table = plt.table(cellText=p[key].values,
                               colLabels=p[key].columns,
                               rowLabels=p[key].index,
                               loc='center',
                               rowColours=rcolors, 
                               colColours=ccolors)
                    table.scale(1,2)
                    table.set_fontsize(14)

        plt.title(self.region)     
        plt.show()


    def plot_density(self):
        plt.figure()
        for key in self.den:
            plt.plot(self.den[key], label=key)

        for key in self.den_pred:
            plt.plot(self.den_pred[key], '-o', label=key+ "_predicted")

        plt.xlabel("time")
        plt.ylabel("density [# vehicles/km]")
        plt.legend()
        plt.show()
        return

    def plot_flow(self):
        plt.figure()
        for key in self.flo:
            plt.plot(self.flo[key], label=key)

        plt.xlabel("time")
        plt.ylabel("flow # vehicles/hr]")
        plt.legend()
        plt.show()
        return

    def plot_input(self):
        # for key in self.inp:
        #     plt.plot(self.inp[key], label=key)
        # plt.show()

        num_input = 5
        fig, axes = plt.subplots(num_input, 1, figsize=(10, 15), sharex=True)
        for controller in self.inp:
            df = self.inp[controller]
            # Plot each column as a separate subplot
            for i, col in enumerate(df.columns):
                axes[i].plot(df.index, df[col], label=controller)
                axes[i].set_ylabel(f" u{i} Actuator {col}")
                axes[i].legend(loc="upper right")

        axes[-1].set_xlabel("Time (0 to 599)") # X-axis label
        plt.tight_layout()
        plt.show()
        return
    
    def plot_metrics(self):
        selected_rows = ["C02_abs", "travel_time", "waiting_time", "nvehicles"]
        for key in self.met:
            df = self.met[key]
            # print(df)
            df_filtered = df.loc[selected_rows]


            rcolors = np.full(len(df_filtered.index), 'linen')
            ccolors = np.full(len(df_filtered.columns), 'lavender')

            table = plt.table(
                                cellText=df_filtered.values,
                                colLabels=df_filtered.columns,
                                rowLabels=df_filtered.index,
                                loc='center',
                                rowColours=rcolors,
                                colColours=ccolors
                             )

            table.scale(1, 2)  # Adjust table size
            table.set_fontsize(14)
        plt.axis("off")
        return
