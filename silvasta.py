import os, sys, json, argparse
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import pandas as pd
import importlib
import time

# Used to estimate the macroscopic fundamental diagram
from src.tasks.estimate import Estimate
from src.simulations.testcontrolsim import test_ControlSim
from src.tasks.dsl import DSL
from src.controllers.controller import Controller
from src.simulations.controlsim import ControlSim

# ----------------------------------------------------------------------------------------------------
from dataclasses import dataclass, asdict
import dataclasses

# ----------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------
from student.mpc_controller import hello
from student.mpc_controller import check_inputs
# ----------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------

taskparams_json = "dep/sumo_files/cocoCity/simparams/cocoCity.json"
with open(taskparams_json, "r") as file:
    taskparams = json.load(file)
mfd_taskparams = "dep/sumo_files/cocoCity/simparams/cocoCity_mfd.json"
with open(mfd_taskparams, "r") as file:
    mfd_taskparams = json.load(file)

# ----------------------------------------------------------------------------------------------------
# FIELD
# ----------------------------------------------------------------------------------------------------

# The code below povides the linear traffic model parameters A, B, C, and d
dsl_task = DSL(taskparams, test_ControlSim)
rho_star = np.array([5.70, 9.83, 10.63, 14.55, 11.94])
v_target = np.ones(5)
sim_period = 20  # sampling time (fixed)
model = dsl_task.simulation.get_model()
A, B, C, d = model.linearize(sim_period, rho_star, v_target)
print(type(A))

# ----------------------------------------------------------------------------------------------------
# FIELD
# ----------------------------------------------------------------------------------------------------


class MPC_Controller(Controller):
    def __init__(self, actuators, params={}) -> None:
        """Initialize the controller"""
        super().__init__(actuators, params)
        self.name = "MPC"
        self.iter = 0
        # self.n_regions = params["n_regions"]
        # self.ul = self.safety[0]  # lower bound on the input = 0.5
        # self.uu = self.safety[1]  # upper bound on the input = 1.5
        # self.params = params
        # print(self.params)
        # print()
        # print(self.safety)
        # print()
        # print(self.m)
        # print()
        # print(self.actuators)
        # print()

    def get_next_input(self):  # type:ignore
        print(f"count controller = {self.iter}")
        self.iter += 1
        pass


# ----------------------------------------------------------------------------------------------------
# FIELD
# ----------------------------------------------------------------------------------------------------


# @dataclasses.dataclass(frozen=True)
# class MPC_Params:
#     g: int = 1
#     Q: int = 2
#     A: np.ndarray = A


@dataclass(frozen=True)
class MPC_Params:
    g: int = 1
    Q: int = 2
    A, B, C, d = model.linearize(sim_period, rho_star, v_target)
    # A: np.ndarray = A


# ----------------------------------------------------------------------------------------------------
# FIELD
# ----------------------------------------------------------------------------------------------------


class MPC_ControlSim(ControlSim):
    def __init__(self, network, taskparams, actuators, controlparams={}):
        super().__init__(
            network=network,
            taskparams=taskparams,
            actuators=actuators,
            controlparams=controlparams,
        )
        self.iter = 0

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
        inputs = {
            # "k": k,  # iteration step, k in range (180)
            # "forecast": forecast,# maybe interesting, mutable?
            # "controller": controller,
            # "controller_name": controller_name,
            # "uAppliedMatrix": uAppliedMatrix,  # u that is sent from here
            # "yMeasuredMatrix": yMeasuredMatrix,  # measured y, filling  up
            # "ySingleStepPredMatrix": ySingleStepPredMatrix,  # y that this function sends
            # "m": m,  # ??? always same, 5
            # "p": p, # ??? always same, 5
            # "r": r,  # rho star, always same
            # "u_min": u_min, # always same, 0.5
            # "u_max": u_max, # always same, 1.5
        }
        check_inputs(inputs)

        print(f"count simulator = {self.iter}")
        self.iter += 1
        # create params
        u = self.controller.get_next_input()
        u = np.ones(5)
        y = np.zeros(5)
        return u, y


# ----------------------------------------------------------------------------------------------------
# FIELD
# ----------------------------------------------------------------------------------------------------


class MPC(Controller):
    def __init__(self, actuators, params={}) -> None:
        """Initialize the controller"""
        super().__init__(actuators, params)
        self.name = "MPC"
        self.iter = 0
        # self.n_regions = params["n_regions"]
        # self.ul = self.safety[0]  # lower bound on the input = 0.5
        # self.uu = self.safety[1]  # upper bound on the input = 1.5
        # self.params = params
        # print(self.params)
        # print()
        # print(self.safety)
        # print()
        # print(self.m)
        # print()
        # print(self.actuators)
        # print()

    def get_next_input(self):  # type:ignore
        print(f"count controller = {self.iter}")
        self.iter += 1
        pass


# ----------------------------------------------------------------------------------------------------
# FIELD
# ----------------------------------------------------------------------------------------------------


def run_mpc():
    print()
    print("Run MPC")
    print()
    hello()

    # params = MPC_Params()
    params = MPC_Params
    print(params.C)

    # print(params.g)
    dsl_task = DSL(taskparams, MPC_ControlSim)
    Mpc = MPC_Controller
    experiment = dsl_task.runtask(
        init_from_notebook=True,
        controller_class=Mpc,
        controller_json=params,
    )


# ----------------------------------------------------------------------------------------------------
# FIELD end
# ----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    run_mpc()
