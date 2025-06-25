from typing import Union, override
from collections import namedtuple

import cvxpy as cvx
import matplotlib.pyplot as plt
import numpy as np
import json

from src.controllers import controller
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

# print(cvx.installed_solvers())
np.set_printoptions(precision=3)


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
            "X_t": cvx.Parameter(self.nx),
            "X_s": cvx.Parameter(self.nx),
            "U_s": cvx.Parameter(self.nu),
        }

        return problem_parameters

    def _get_constraints(self) -> list[cvx.Constraint]:
        """
        Define cvx constraints from dynamics and system
        """
        # X,U time series
        X = self.variables["X"]
        U = self.variables["U"]
        # initial x and disturbance for 1 iteration
        X_t = self.cvx_parameters["X_t"]
        # steady state prediction for 1 iteration
        X_s = self.cvx_parameters["X_s"]
        U_s = self.cvx_parameters["U_s"]

        A = self.A
        B = self.B
        # constraint for first state
        constraints = [X[:, 0] == X_t]
        for k in range(self.K):
            constraints += [
                # dynamic constraints
                X[:, k + 1] == A @ X[:, k] + B @ U[:, k],
                # state constraints
                #
                # input constraints
                U[:, k] <= 1.5 * np.ones(self.nu) - U_s,
                U[:, k] >= 0.5 * np.ones(self.nu) + U_s,
            ]
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
        # try to solve
        self.problem.solve(verbose=True)
        # solver=cvx.MOSEK, verbose=False
        # check solver status
        for key, variable in self.variables.items():
            if variable.value is None:
                print(f"Solver failed: {key}.value is None")
                # raise ValueError(f"Solver failed: {key}.value is None")

    def get_u_delta(self, x_t, x_s, u_s):
        """
        Calls solver, takes first input or 0, predicts next state
        """
        self.cvx_parameters["X_t"].value = x_t
        self.cvx_parameters["X_s"].value = x_s
        self.cvx_parameters["U_s"].value = u_s

        self.solve_problem()
        # get input
        if self.variables["U"].value is not None:
            u_delta = self.variables["U"].value[:, 0]
        else:
            u_delta = np.zeros(self.nx)
            print("u_delta was not calculated correctly!")
        return u_delta

    def get_next_input(self, *args):
        return super().get_next_input(*args)


class MPC_ControlSim(ControlSim):
    def __init__(self, network, taskparams, actuators, controlparams={}):
        super().__init__(
            network=network,
            taskparams=taskparams,
            actuators=actuators,
            controlparams=controlparams,
        )
        self.output_path = "out/mpc/"
        # steady state and input
        self.X_s: cvx.Variable
        self.U_s: cvx.Variable
        # disturbance (summary of C @ q + d), mutable
        self.D_t: cvx.Parameter
        # optimal value, designed mutable
        self.Rho: cvx.Parameter
        self.problem: cvx.Problem

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

        if k == 0:
            self.setup_steady_state_OP()
        print()
        print(f"Iteration: {k}")
        # extract current state
        x = yMeasuredMatrix[:, k]
        # calculate disturbance
        q = forecast[k, :]
        disturbance = self.controller.C @ q + self.controller.d
        # prepare (steady) state for delta zero system
        x_delta, x_s, u_s = self.target_selector(x, r, disturbance)
        # get input for delta zero system
        u_delta = self.controller.get_u_delta(x_delta, x_s, u_s)
        u = u_delta + u_s
        # apply dynamics to get next state
        y = self.predict_next_state(x, u, disturbance)
        print(f"{k}: u = {u}, y = {y}")
        # u = np.ones(5)
        # y = np.zeros(5)
        input()
        return u, y

    def predict_next_state(self, x, u, d):
        """
        Applies dynamics
        """
        A = self.controller.A
        B = self.controller.B
        x_next = A @ x + B @ u + d

        return x_next

    def target_selector(self, x, r, disturbance):
        """
        Calculates steady state and delta x
        """
        print(disturbance)
        self.D_t.value = disturbance.reshape(-1)
        self.Rho.value = r
        self.problem.solve(solver="SCS")  # MOSEK no license...
        x_s = self.X_s.value
        # x_s = X_s if X_s is not None else r #TODO: failsave
        u_s = self.U_s.value
        x_delta = x - x_s
        print(f"x_s: {x_s}, u_s: {u_s}")
        print(f"x_d: {x_delta}")
        return x_delta, x_s, u_s

    def setup_steady_state_OP(self):
        """
        Initialization of steady state optimization problem
        """
        nx, nu = self.controller.nx, self.controller.nu
        X_s = cvx.Variable(nx)
        U_s = cvx.Variable(nu)
        D_t = cvx.Parameter(nx)
        Rho = cvx.Parameter(nx)
        A = self.controller.A
        B = self.controller.B
        constraints = [
            X_s == A @ X_s + B @ U_s + D_t,
            U_s <= 1.5 * np.ones(self.controller.nu),
            U_s >= 0.5 * np.ones(self.controller.nu),
        ]
        objective = cvx.Minimize(cvx.norm(X_s - Rho))
        problem = cvx.Problem(objective, constraints)
        # finally set all states
        self.X_s = X_s
        self.U_s = U_s
        self.D_t = D_t
        self.Rho = Rho
        self.problem = problem


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
        "K": 90,
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
