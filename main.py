# %env SUMO_HOME=/usr/share/sumo
# DO NOT TOUCH THE ABOVE LINE: needed for SUMO, which supports the traffic simulation

import os, sys, json, argparse
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import pandas as pd
import importlib
import time

# import necessary modules # src is short for "source code", and is a folder that you are given

# The following package is used to estimate the macroscopic fundamental diagram
# Used to estimate the macroscopic fundamental diagram
from src.tasks.estimate import Estimate

# (helper function, can be ignored)
from src.simulations.testcontrolsim import test_ControlSim

# The following packages are for simulation and control design
# DSL is the class used for Dynamic Speed Limit control which runs a ControlSim simulation with Controller in the loop
from src.tasks.dsl import DSL

# An abstract class for the controllers implemented which determine the dynamic speed limits
from src.controllers.controller import Controller

# A simulation environment in which we test our controllers
from src.simulations.controlsim import ControlSim  # type:ignore

# The following packages are for visualization
from src.visualization.visualization import *
from src.data.comparison import Comparison
from src.data.experiment import Experiment

# Load the parameters of the simulation
taskparams_json = "dep/sumo_files/cocoCity/simparams/cocoCity.json"
with open(taskparams_json, "r") as file:
    taskparams = json.load(file)
mfd_taskparams = "dep/sumo_files/cocoCity/simparams/cocoCity_mfd.json"
with open(mfd_taskparams, "r") as file:
    mfd_taskparams = json.load(file)

# ----------------------------------------------------------------------------------------------------
# Optimal Density
# ----------------------------------------------------------------------------------------------------

rho_star = np.array([5.70, 9.83, 10.63, 14.55, 11.94])
n_regions = 5

optimal_density = {
    f"Region {region}": round(rho_star[region], 2) for region in range(n_regions)
}
print("Optimal Densities:\n", json.dumps(optimal_density, indent=4))


# ----------------------------------------------------------------------------------------------------
# Spawning Vehicles
# ----------------------------------------------------------------------------------------------------

# The pd.read_csv() line below gives io_data, which can be used as data for your data-driven controller
io_data = pd.read_csv("./dep/sumo_files/cocoCity/control/edge/io_data.csv")
# this line prints the column names of the training data
print(f"Data Columns:\n{io_data.columns.values}")

# the following code produces the evaluation scenario plot above
spawnedVehicles = np.load("dep/sumo_files/cocoCity/routing/spawning_training.npy")
fig, axes = plt.subplots(5, 1, figsize=(8, 6), sharex=True)
for region in range(5):
    axes[region].plot(
        spawnedVehicles[:, region],
        color="blue",
        label=f"Vehicles spawned in Region {region}",
    )
    axes[region].set_ylabel("# Vehicles")
    axes[region].legend()
axes[4].set_xlabel("Simulation Time")

plt.suptitle("Spawned Vehicles in Each Region")
plt.tight_layout()
plt.savefig("spawning_training.png", dpi=300, bbox_inches="tight")
plt.show()

# ----------------------------------------------------------------------------------------------------
# Model of the System
# ----------------------------------------------------------------------------------------------------

# The code below povides the linear traffic model parameters A, B, C, and d
dsl_task = DSL(taskparams, test_ControlSim)
rho_star = np.array([5.70, 9.83, 10.63, 14.55, 11.94])
v_target = np.ones(5)
sim_period = 20  # sampling time (fixed)
model = dsl_task.simulation.get_model()
A, B, C, d = model.linearize(sim_period, rho_star, v_target)

print("Matrix A:")
print(A)
print("Matrix B:")
print(B)
print("Matrix C:")
print(C)
print("Vector d:")
print(d)

# ----------------------------------------------------------------------------------------------------
# Evaluation scenario
# ----------------------------------------------------------------------------------------------------

# the following code produces the evaluation scenario plot above
plot = False
# plot = True
if plot:
    spawnedVehicles = np.load("dep/sumo_files/cocoCity/routing/spawning_evaluation.npy")
    fig, axes = plt.subplots(5, 1, figsize=(8, 6), sharex=True)
    for region in range(5):
        axes[region].plot(
            spawnedVehicles[:, region],
            color="blue",
            label=f"Vehicles spawned in Region {region}",
        )
        axes[region].set_ylabel("# Vehicles")
        axes[region].legend()
    axes[-1].set_xlabel("Simulation Time")

    plt.suptitle("Spawned Vehicles in Each Region")
    plt.tight_layout()
    plt.show()

# ----------------------------------------------------------------------------------------------------
# noControl_controller
# ----------------------------------------------------------------------------------------------------


class noControl_controller(Controller):
    def __init__(self, actuators, params={}) -> None:
        """Initialize the controller"""
        super().__init__(actuators=actuators, params=params)
        self.name = "noControl_controller"
        self.example = params["Example"]

    def get_next_input(self):  # type:ignore
        """
        [CONTROLLER DESCRIPTION]
        -------------------------------
        Inputs: IMPLEMENT
        Outputs: IMPLEMENT
        """
        u = np.ones(5)  # one input for each DSL
        y = np.zeros(5)  # one output prediction for each region
        return u, y


class ControlSim(ControlSim):  # type:ignore
    def __init__(self, network, taskparams, actuators, controlparams={}):
        super().__init__(
            network=network,
            taskparams=taskparams,
            actuators=actuators,
            controlparams=controlparams,
        )
        self.output_path = "out/nococo/"

    def compute_input(
        self,
        k,
        forecast,
        controller,
        controller_name,
        uAppliedMatrix,
        yMeasuredMatrix,
        ySingleStepPredMatrix,
        m,
        p,
        rho_opt,
        u_min,
        u_max,
    ):
        """
        Compute the input for the controller.

        This function is called "under the hood" of an Unjam traffic simulation/a dsl_task. Thus, the inputs to compute_input are provided automatically.

        COCO City has set up the code so that the following inputs are provided to compute_input for you:
        - forecast, which provides the forecast of the spawned vehicles to be used for predictive control (though some engineering is required to produce y_future),
        - uAppliedMatrix and yMeasuredMatrix are the matrices of inputs and outputs filled in over the course of a simulation. They can be used as the lead-in data for a data-driven controller (though some engineering is required).
        - ySingleStepPredMatrix is the matrix of single step predictions ySingleStepPred, filled in over the course of a simulation.
        -------------------------------
        **Inputs:
        k: int, discrete time step (each discrete time step corresponds to 20 seconds of wall-clock time)
        forecast: np.array, forecast of spawned vehicles from all 5 regions (including center region) to the center region
        controller: Controller, controller object, defined separately
        controller_name: str, name of the controller
        uAppliedMatrix: np.array, memory of applied control inputs (zeros for timesteps that have not occurred yet)
        yMeasuredMatrix: np.array, memory of outputs (zeros for timesteps that have not occurred yet)
        ySingleStepPredMatrix: np.array, memory of single-step predicted outputs (zeros for timesteps that have not occurred yet)
        m: int, number of actuators
        p: int, number of outputs (1 per region)
        rho_opt: np.array((5,)), optimal density for each of the 5 regions
        u_min: np.array, minimum control input for each actuator (0.5)
        u_max: np.array, maximum control input for each actuator (1.5)
        -------------------------------
        **Returns:
        uApplied: np.array, control input for each actuator (5 actuators) (automatically put into uAppliedMatrix)
        ySingleStepPred: np.array, predicted density for each region (5 regions) (automatically put into ySingleStepPredMatrix)
        """

        uApplied, ySingleStepPred = controller.get_next_input()

        return uApplied, ySingleStepPred


# for the no control case, no parameters are needed. Below is just an example of how a parameter could be set.
noControl_control_params = {"Example": 5}
print(ControlSim)
# takes around 30s to run
dsl_task = DSL(
    taskparams, ControlSim
)  # DSL is a class with a "runtask" function, "dsl_task" is an instance of the class
start_time = time.time()
experiment = dsl_task.runtask(
    init_from_notebook=True,
    controller_class=noControl_controller,
    controller_json=noControl_control_params,
)
end_time = time.time()
run_time = end_time - start_time
print(time)
print("No-controller time:", run_time)

# this line will instantiate a SUMO simulation and the specified controller
# The controller is determined by "controller_class" and the parameters are determined by "controller_json"
# runtask starts SUMO (the traffic sim)
# "experiment" is the saved output of the simulation

# ----------------------------------------------------------------------------------------------------
# Control evaluation tool
# ----------------------------------------------------------------------------------------------------

region = "Region 4"
com = Comparison([experiment], ["Current Status"], region=region)  # type:ignore

com.plot_density()
com.plot_flow()
com.plot_input()
com.plot_metrics()

## Alternatively, if you have the saved output_dir, you can also plot the results.
# This is where the output of the experiment was saved.
output_dir = experiment.info["output_path"]  # type:ignore
experiment_saved = Experiment()  # instantiate an empty experiment
experiment_saved.load(output_dir)  # Load in the simulation experiment result
com = Comparison([experiment_saved], ["Current Status (saved)"], region=region)  # type:ignore
com.plot_metrics()

# ----------------------------------------------------------------------------------------------------
# Gif generation
# ----------------------------------------------------------------------------------------------------

gif = False
gif = True
if gif:
    start_time = time.time()
    # GIF Generation
    # [DO NOT TOUCH THE LINE BELOW] Turns off matplotlib in-line plotting to save memory, needed for GIF generation.
    # %matplotlib agg

    # This takes around 2 minutes (can be commented out)

    # This is where the output of the experiment was saved.
    output_dir = experiment.info["output_path"]  # type:ignore
    # Specify where to save the density git file, can change to your own path
    output_gif_path = "figs/no_control_demo_heatmap.gif"
    cmap, norm = cocoCity_plot_generate_density_gif(output_dir, output_gif_path)

    # Display saved GIF
    # display(Image(url=output_gif_path))
    # [DO NOT TOUCH THE LINE BELOW] turns matplotlib in-line back on.
    plot_color_legend(cmap, norm)

    # %matplotlib inline
    end_time = time.time()
    run_time = end_time - start_time
    print("GIF time:", run_time)
# ----------------------------------------------------------------------------------------------------
# coco P Controller
# ----------------------------------------------------------------------------------------------------


# First, the P controller and the ControlSim is built
class pController(Controller):
    def __init__(self, actuators, params={}) -> None:
        """Initialize the controller"""
        super().__init__(actuators, params)
        self.name = "P"
        self.n_regions = params["n_regions"]

        self.ul = self.safety[0]  # lower bound on the input = 0.5
        self.uu = self.safety[1]  # upper bound on the input = 1.5

        self.Kp = params["Kp"]

    def get_next_input(self, n, target_region, r):  # type:ignore
        """Derive and check the inputs from the optimization
        Args:
            n: Current densities
            T: Current time
        Returns:
            Next inputs
        """
        self.r = r.copy()
        target_region_index = target_region - 1
        n = n.copy()[target_region_index]
        r = self.r.copy()[target_region_index]

        error = r - n
        u = 1 + self.Kp * error

        u = np.clip(u, self.ul, self.uu)

        return np.tile(u, 5)


class pControl_ControlSim(ControlSim):
    def __init__(self, network, taskparams, actuators, controlparams={}):
        super().__init__(
            network=network,
            taskparams=taskparams,
            actuators=actuators,
            controlparams=controlparams,
        )
        self.output_path = "out/pcoco/"

    def compute_input(  # type: ignore
        self,
        k,
        forecast,
        controller,
        controller_name,
        uAppliedMatrix,
        yMeasuredMatrix,
        ySingleStepPredMatrix,
        m,
        p,
        r,
        u_min,
        u_max,
    ):
        freq = 5
        if k % freq == 0:
            n = yMeasuredMatrix[:, k]
            target_region = 5
            u = controller.get_next_input(n, target_region, r)
            y = np.zeros(5)  # No prediction, just assume it's flat
            return u, y
        else:
            return uAppliedMatrix[k - 1, :], ySingleStepPredMatrix[k - 1, :]


# Second, the Kp parameter is set and the P controller is simulated on the evaluation sim
pControl_control_params = {"Kp": 0.01}

dsl_task = DSL(
    taskparams, pControl_ControlSim
)  # DSL is a class with a "runtask" function, "dsl_task" is an instance of the class
controller_class = pController
controller_json = pControl_control_params

start_time = time.time()
experiment = dsl_task.runtask(
    init_from_notebook=True,
    controller_class=controller_class,
    controller_json=controller_json,
)
end_time = time.time()
run_time = end_time - start_time
print("P-controller time:", run_time)
# runtask starts SUMO (the traffic sim)
# "experiment" is the saved output of the simulation


# Third, the relevant metrics and timeseries are plotted

region = "Region 4"
com = Comparison([experiment], ["Current Status"], region=region)  # type:ignore

com.plot_density()
com.plot_flow()
com.plot_input()
com.plot_metrics()
