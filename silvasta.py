from typing import Union, override
from collections import namedtuple

import cvxpy as cvx
import matplotlib.pyplot as plt
import numpy as np
import json

from src.tasks.estimate import Estimate
from src.simulations.testcontrolsim import test_ControlSim
from src.tasks.dsl import DSL
from src.controllers.controller import Controller
from src.simulations.controlsim import ControlSim  # type:ignore

# from src.visualization.visualization import *
from src.data.comparison import Comparison
from src.data.experiment import Experiment

# -------------------------------------------------------------------------------------------------
taskparams_json = "dep/sumo_files/cocoCity/simparams/cocoCity.json"
with open(taskparams_json, "r") as file:
    taskparams = json.load(file)
mfd_taskparams = "dep/sumo_files/cocoCity/simparams/cocoCity_mfd.json"
with open(mfd_taskparams, "r") as file:
    mfd_taskparams = json.load(file)


# -------------------------------------------------------------------------------------------------
# FIELD
# -------------------------------------------------------------------------------------------------
# The code below povides the linear traffic model parameters A, B, C, and d
def get_system_params():
    dsl_task = DSL(taskparams, test_ControlSim)
    rho_star = np.array([5.70, 9.83, 10.63, 14.55, 11.94])
    v_target = np.ones(5)
    sim_period = 20  # sampling time (fixed)
    model = dsl_task.simulation.get_model()
    A, B, C, d = model.linearize(sim_period, rho_star, v_target)
    return (A, B, C, d)

    # -------------------------------------------------------------------------------------------------
    # FIELD
    # -------------------------------------------------------------------------------------------------


class MPC_Controller(Controller):
    def __init__(self, actuators, params={}) -> None:
        """Initialize the controller"""
        super().__init__(actuators=actuators, params=params)
        self.name = "MPC"
        # ---
        self.output_path = "./test"
        ### system specific params
        # state space model
        self.A: np.ndarray
        self.B: np.ndarray
        self.C: np.ndarray
        # linearization offset
        self.d: np.ndarray
        # dimenstions
        self.nx: int
        self.nu: int
        # cost parameter
        self.Q: np.ndarray
        self.R: np.ndarray
        ### task specific params
        # initial condition, gets updated in simulation
        self.x_t: np.ndarray
        # prediciton horizon
        self.T: int
        # control horizon
        self.K: int
        # check, transform and set params
        self._set_system(params)
        ### optimization problem
        self.variables = self._get_variables()
        self.cvx_parameters = self._get_cvx_parameters()
        constraints = self._get_constraints()
        objective = self._get_objective()
        self.problem = cvx.Problem(objective, constraints)
        print("MPC controller initialized")
        # placeholder for storing information
        self.summed_cost: float = 0
        self.x_trajectory: np.ndarray
        self.u_trajectory: np.ndarray

    def _set_system(self, p):
        # system parameter
        A, B, C, d = get_system_params()
        self.A = np.atleast_2d(A)
        n_h, n_w = self.A.shape
        self.nx = n_w
        self.B = np.atleast_2d(B)
        n_h_check, p_w = self.B.shape
        self.nu = p_w
        self.C = np.atleast_2d(C)
        self.d = np.atleast_2d(d)
        # cost parameter
        self.Q = np.atleast_2d(np.array(p["Q"]))
        self.R = np.atleast_2d(np.array(p["R"]))
        # horizon params
        self.T = p["T"]
        self.K = p["K"]
        # target state

    def set_current_state(self, x, q):
        self.x_t = np.array(x)
        self.q_t = np.array(q)

    def _get_variables(self) -> dict:
        """
        Define and store optimization variables
        """
        variables = {
            "X": cvx.Variable((self.nx, self.K + 1)),
            "U": cvx.Variable((self.nu, self.K)),
        }
        return variables

    def _get_cvx_parameters(self) -> dict:
        """
        Define mutable problem parameters
        """
        problem_parameters = {
            "current_state": cvx.Parameter(self.nx),
            "current_spawning": cvx.Parameter(self.nx),
        }

        return problem_parameters

    def _get_constraints(self) -> list[cvx.Constraint]:
        """
        Define cvx constraints from dynamics and system
        """
        # state and input for optimization
        X = self.variables["X"]
        U = self.variables["U"]
        # constraint with mutable parameter (receding horizon)
        constraints = [X[:, 0] == self.cvx_parameters["current_state"]]
        q_t = self.cvx_parameters["current_spawning"]
        for k in range(self.K):
            # dynamics
            constraints += [
                X[:, k + 1]
                == self.A @ X[:, k] + self.B @ U[:, k] + self.C @ q_t + self.d
            ]
            # state constraints
            #
            # input constraints
            constraints += [cvx.norm(U[:, k], 1) <= 0.5]
        # final state constraint
        constraints += [X[:, self.K] == 0]

        return constraints

    def _get_objective(self) -> Union[cvx.Minimize, cvx.Maximize]:
        """
        Define objective
        """
        X = self.variables["X"]
        U = self.variables["U"]
        objective = 0
        for k in range(self.K):
            objective += cvx.quad_form(X[:, k], self.Q) + cvx.quad_form(U[:, k], self.R)

        return cvx.Minimize(objective)

    def solve_problem(self):
        """
        Updates parameter, runs solver and checks result
        """
        # refresh solver parameter
        self.cvx_parameters["current_state"].value = self.x_t
        self.cvx_parameters["current_spawning"].value = self.q_t
        # try to solve
        self.problem.solve(verbose=True)
        # solver=cvx.MOSEK, verbose=False
        # check solver status
        for key, variable in self.variables.items():
            if variable.value is None:
                print(f"Solver failed: {key}.value is None")
                # raise ValueError(f"Solver failed: {key}.value is None")

    def get_next_input(self):  # type:ignore
        """
        Calls solver, takes first input or 0, predicts next state
        """
        self.solve_problem()
        # get input
        try:
            u = self.variables["U"].value[:, 0]
        except ValueError:
            print("fail!!! u ist not properly calculated")
            u = [0, 0, 0, 0, 0]

        return u

    def predict_next_state(self, x, u, q):
        """
        Applies dynamics
        """
        x_next = self.A @ x + self.B @ u + self.C @ q + self.d
        return x_next


class MPC_ControlSim(ControlSim):
    def __init__(self, network, taskparams, actuators, controlparams={}):
        super().__init__(
            network=network,
            taskparams=taskparams,
            actuators=actuators,
            controlparams=controlparams,
        )
        self.output_path = "out/mpc/"

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
            # "forecast": forecast,# maybe interesting, mutable?(360, 5)
            # for i, f in enumerate(forecast): always 1 from 180:360
            #     print(f"{i} - {f}")
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
        print()
        print(f"Iteration: {k}")
        # extract current state and vehicles
        x_t = yMeasuredMatrix[:, k]
        q_t = forecast[k, :]
        # prepare state for delta zero system
        x_t_delta = x_t - r
        self.controller.set_current_state(x_t_delta, q_t)
        # get input for delta zero system
        u_delta = self.controller.get_next_input()
        u = u_delta + [1, 1, 1, 1, 1]
        # apply dynamics to get next state
        y = self.controller.predict_next_state(x_t, u, q_t)
        print(f"{k}: u = {u}, y = {y}")
        # u = np.ones(5)
        # y = np.zeros(5)
        input()
        return u, y


# -------------------------------------------------------------------------------------------------
# FIELD
# -------------------------------------------------------------------------------------------------


def run_mpc():
    print()
    print("Run MPC")
    print()

    rho_star = [5.70, 9.83, 10.63, 14.55, 11.94]

    # q_star = [1 / q**2 for q in rho_star]
    # print(q_star)

    q_star = [0, 0, 0, 0, 1]

    param = {
        "Q": np.diag(q_star),
        "R": np.eye(5),
        "T": 0,
        "K": 30,
        "rho_star": rho_star,
    }
    dsl_task = DSL(taskparams, MPC_ControlSim)

    experiment = dsl_task.runtask(
        init_from_notebook=True,
        controller_class=MPC_Controller,
        controller_json=param,
    )
    region = "Region 4"
    com = Comparison([experiment], ["Current Status"], region=region)  # type:ignore

    com.plot_density()
    com.plot_flow()
    com.plot_input()
    com.plot_metrics()


# -------------------------------------------------------------------------------------------------
# FIELD end
# -------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    run_mpc()
