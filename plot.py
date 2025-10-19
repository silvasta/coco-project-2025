from pathlib import Path
from sys import exec_prefix
from typing import Union, override
from collections import namedtuple
import cvxpy as cvx
import matplotlib.pyplot as plt
import numpy as np
import json

from src.controllers import controller

# from src.data import experiment
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
cfg.set_tbl_rows(90)

BEST_RESULTS = [
    # "q_star_final_S_500_K_60",
    # "q_star_final_S_500_K_62",
    # "q_star_final_S_470_K_60",
    # "q_star_final_S_470_K_62",
    # "q_star_final_S_600_K_60",
    # "q_star_final_S_600_K_62",
    "q_star_S_500_K_60",
    "august_S_500_K_60",
]

# WARN: path management!
project_root = Path.cwd()


def plot_best():
    flows = pl.DataFrame()
    densities = pl.DataFrame()
    for p in BEST_RESULTS:
        experiment = project_root / "out/mpc" / p
        print(experiment)
        # density
        result_type = "density_results.csv"
        result_path = f"{experiment}/results/{result_type}"
        df = pl.read_csv(result_path)
        density = df.select(pl.nth(5)).rename(lambda column_name: p)  # type:ignore
        densities = densities.with_columns(density)
        # flow
        result_type = "flow_results.csv"
        result_path = f"{experiment}/results/{result_type}"
        df = pl.read_csv(result_path)
        flow = df.select(pl.nth(5)).rename(lambda column_name: p)  # type:ignore
        flows = flows.with_columns(flow)
    # density
    for col in densities.columns:
        plt.plot(np.arange(180), densities[col], label=col)
    plt.title("Density")
    plt.legend(BEST_RESULTS)
    plt.show()
    plt.close()
    # flow
    for col in flows.columns:
        plt.plot(np.arange(180), flows[col], label=col)
    plt.title("Flow")
    plt.legend(BEST_RESULTS)
    plt.show()
    plt.close()


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

        # use this for metrics
        result = df.select(pl.nth(1)).rename(lambda column_name: experiment_name)
        # use this for all others
        # result = df.select(pl.nth(5)).rename(lambda column_name: experiment_name)

        summary = summary.with_columns(result)

    legend = legend.to_series().to_list()
    statistic = summary.drop("Legend").transpose(
        include_header=True, column_names=legend
    )
    # debug or full list
    # print(summary)
    # print(statistic)

    # ## flow**.csv
    # stat = "Region 4"
    # print(f"stat: {stat}")
    # print(statistic.sort(stat))

    ## metrics.csv
    metrics = [
        "column",
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
        print(statistic.select(metrics).sort(stat))


if __name__ == "__main__":
    # plot_mpc_results()
    # compare_mpc_results()
    plot_best()
