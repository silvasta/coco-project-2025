from typing import Union, override
from collections import namedtuple
import cvxpy as cvx
import matplotlib.pyplot as plt
import numpy as np
import json

from src.controllers import controller
from src.tasks import dsl
from src.tasks.estimate import Estimate
from src.simulations.testcontrolsim import test_ControlSim
from src.tasks.dsl import DSL
from src.controllers.controller import Controller
from src.simulations.controlsim import ControlSim  # type:ignore

# from src.visualization.visualization import *
from src.data.comparison import Comparison
from src.data.experiment import Experiment

# -------------------------------------------------------------------------------------------------
### DEVELOPEMENT
import polars as pl
import glob
# -------------------------------------------------------------------------------------------------

cfg = pl.Config()
cfg.set_tbl_rows(100)

BEST_RESULTS = [
    # Travel time
    "q_star_S_500_K_60",
    "q_star_S_470_K_62",
    "q_star_S_600_K_62",
    "q_star_S_520_K_62",
    # Waiting time
]


def plot_mpc_results():
    to_plot = [
        "quite_good_mpc",
        "mpcq_star_S_500_K_60",
    ]
    for p in to_plot:
        output_path = f"/home/silvan/coco/out/{p}"
        plot_results(output_path)


def plot_results(result_path):
    region = "Region 4"
    print(f"plot {result_path} for region {region}")

    experiment = Experiment()
    experiment.load(result_path)
    com = Comparison([experiment], ["Current Status (saved)"], region=region)  # type:ignore

    com.plot_density()
    com.plot_flow()
    com.plot_input()
    com.plot_metrics()


def compare_mpc_results():
    summary = pl.DataFrame()
    for experiment in glob.glob("/home/silvan/coco/out/mpc/*"):
        experiment_name = experiment.split("/")[-1]

        # result_type = "density_prediction_results.csv"
        # result_type = "density_results.csv"
        # result_type = "error_results.csv"
        # result_type = "flow_results.csv"
        # result_type = "input_results.csv"
        result_type = "metrics.csv"

        result_path = f"{experiment}/results/{result_type}"

        df = pl.read_csv(result_path)

        if summary.shape[0] == 0:
            legend = df.select(pl.nth(0)).rename(lambda column_name: "Legend")  # type:ignore
            summary = summary.with_columns(legend)

        result = df.select(pl.nth(1)).rename(lambda column_name: experiment_name)
        # result = df.select(pl.nth(5)).rename(lambda column_name: experiment_name)
        summary = summary.with_columns(result)

    legend = legend.to_series().to_list()
    statistic = summary.drop("Legend").transpose(
        include_header=True, column_names=legend
    )
    # print(summary)
    # print(statistic)

    # ## flow**.csv
    # stat = "Region 4"
    # print(f"stat: {stat}")
    # print(statistic.sort(stat))

    ## metrics.csv
    metrics = [
        "travel_time",
        "waiting_time",
        # "C0_abs",
        "C02_abs",
        # "HC_abs",
        # "PMx_abs",
        # "NOx_abs",
        "fuel_abs",
        "nvehicles",
    ]
    for stat in metrics:
        print()
        print(f"stat: {stat}")
        print(statistic.sort(stat))


if __name__ == "__main__":
    # plot_mpc_results()
    compare_mpc_results()
