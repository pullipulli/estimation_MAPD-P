"""
This module contains the StatsVisualizer class that allows to visualize the statistics of the simulations.
author: Andrea Pullia (@pullipulli)
"""
import os
from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
import seaborn as sns
from matplotlib.axes import Axes

import RootPath
from Utils.type_checking import MapOutput, RunId, MapName

TIME_METRIC_NAMES = ["costs", "serv_times", "pickup_to_goal_times", "start_to_pickup_times", "runtimes", "makespans", "earth_mover_dist"]
OTHER_METRICS = ["earth_mover_dist"]
TIME_METRIC_LABELS = ["Costs per Task", "Service Times [s]", "Pickup to Goal Times [s]", "Start to Pickup Times[s]", "Runtimes [s]",
                      "Makespans", "Earth Mover Distance"]
MAX_TASK_TIME_NAMES = ["earth_mover_dist"]
ONLY_LEARNING_TIME_NAMES = ["earth_mover_dist"]

TIME_METRICS = dict(zip(TIME_METRIC_NAMES, TIME_METRIC_LABELS))

TIME_EVOLUTION_NAMES = ["serv_times", "pickup_to_goal_times", "start_to_pickup_times", "runtimes", "earth_mover_dist"]


class StatsVisualizer:
    """
    Class that allows to visualize the statistics of the simulations.
    """
    def __init__(self, maps: list[MapOutput], tasks_num: list[int], task_frequency_num: list[float], td_update_num: list[int], save=True):
        self.maps = maps
        self.map_names = set()
        for map in self.maps:
            config = self.stats_of(map)[0]
            self.map_names.add(config["map"])

        self.params = {
            "agents": [],
            "tasks": tasks_num,
            "task_frequency": task_frequency_num,
            "pickup": [],
            "goal": [],
            "td_update": td_update_num
        }

        self.pixel_size = 1/plt.rcParams['figure.dpi']

        self.variable_param = None # At the beginning, no variable parameter is set because it is not known yet

        self.padding = 0
        self.fontSize = 12
        self.save = save

    def check_for_variable_param(self):
        variable_params = []
        for param in self.params:
            if len(self.params[param]) > 1:
                variable_params.append(param)
        assert len(variable_params) == 1, "Only one parameter can be variable"
        self.variable_param = variable_params[0]

    def get_map_names(self):
        """Get the names of the maps."""
        return self.map_names

    def get_variable_config_parameter(self):
        """Get the variable parameter of the configuration."""
        return self.variable_param

    def stats_of(self, map_dict: MapOutput = None, run_id: RunId = None):
        """Get the statistics of a simulation run."""
        if run_id is not None:
            map_dict = [map for map in self.maps if map["run_id"] == run_id][0]

        fixed = map_dict["fixed"]
        learning = map_dict["learning"]

        time = {
            "fixed": np.arange(0, len(fixed["costs"]), 1),
            "learning": np.arange(0, len(learning["costs"]), 1)
        }

        df_time_fixed = pd.DataFrame({
            "costs": fixed["costs"], "serv_times": fixed["serv_times"],
            "pickup_to_goal_times": fixed["pickup_to_goal_times"],
            "start_to_pickup_times": fixed["start_to_pickup_times"], "runtimes": fixed["runtimes"],
            "makespans": time["fixed"], "earth_mover_dist": fixed["earth_mover_dist"],
        }, index=time["fixed"])

        df_time_learning = pd.DataFrame({
            "costs": learning["costs"], "serv_times": learning["serv_times"],
            "pickup_to_goal_times": learning["pickup_to_goal_times"],
            "start_to_pickup_times": learning["start_to_pickup_times"], "runtimes": learning["runtimes"],
            "makespans": time["learning"], "earth_mover_dist": learning["earth_mover_dist"],
        }, index=time["learning"])

        df_tasks_fixed = pd.DataFrame({
            "estimated": fixed["estimated_costs"], "real": fixed["real_costs"]
        })

        df_tasks_learning = pd.DataFrame({
            "estimated": learning["estimated_costs"], "real": learning["real_costs"]
        })

        assert fixed['agents'] == learning['agents'] and fixed['tasks'] == learning['tasks'] and fixed['map_name'] == \
               learning[
                   "map_name"] and fixed['task_frequency'] == learning['task_frequency'] and fixed['pickup'] == \
               learning['pickup'] and \
               fixed['goal'] == learning['goal'] and fixed['task_distr_update'] == learning['task_distr_update']

        config = {
            "agents": fixed["agents"], "tasks": fixed["tasks"], "map": fixed["map_name"],
            "pickup": fixed["pickup"], "goal": fixed["goal"], "task_frequency": fixed["task_frequency"],
            "td_update": fixed["task_distr_update"],
            "last_task_time": max(fixed["last_task_time"], learning["last_task_time"])
        }

        return config, df_time_fixed, df_time_learning, df_tasks_fixed, df_tasks_learning, fixed["traffic"], learning[
            "traffic"]

    def get_run_ids_from_map(self, map_name: MapName):
        """Get the run ids of the simulations of a map."""
        run_ids = []
        for map in self.maps:
            config = self.stats_of(map)[0]
            if config["map"] == map_name:
                run_ids.append(map["run_id"])
        return run_ids

    def show_double_bar_time_metric(self, map_name: MapName):
        """
        Show the double bar plot of the time metrics.
        :param map_name: The name of the map to show the metrics of.
        """
        run_ids = self.get_run_ids_from_map(map_name)

        # set width of bar
        barWidth = 0.25
        possible_variable_num = []
        fixed_metric = defaultdict(lambda: [])
        learning_metric = defaultdict(lambda: [])
        variable_param = self.get_variable_config_parameter()

        for run_id in run_ids:
            config, df_fixed, df_learning = self.stats_of(run_id=run_id)[0:3]
            for metric_name in TIME_METRIC_NAMES:
                metric = df_fixed[metric_name].iloc[-1]
                fixed_metric[metric_name].append(metric)
                metric_learning = df_learning[metric_name].iloc[-1]
                learning_metric[metric_name].append(metric_learning)

            if config[variable_param] not in possible_variable_num:
                possible_variable_num.append(config[variable_param])

        config = self.stats_of(run_id=run_ids[0])[0]

        for metric_name in TIME_METRIC_NAMES:
            fig, ax = plt.subplots(figsize=(500*self.pixel_size, 500*self.pixel_size))
            number_of_bars_per_group = 2

            if metric_name == "costs":
                fixed_metric[metric_name] = [x / config["tasks"] for x in fixed_metric[metric_name]]
                learning_metric[metric_name] = [x / config["tasks"] for x in learning_metric[metric_name]]

            bar0 = np.arange(len(fixed_metric[metric_name]))
            bar1 = [x + barWidth for x in bar0]

            if metric_name in ONLY_LEARNING_TIME_NAMES:
                number_of_bars_per_group = 1

            if metric_name not in ONLY_LEARNING_TIME_NAMES:
                ax.barh(bar0, fixed_metric[metric_name], color='r', height=barWidth,
                                               edgecolor='grey', label='TP-mp')
            ax.barh(bar1, learning_metric[metric_name], color='g', height=barWidth,
                                           edgecolor='grey', label='TP-mp\nwith Estimation')
            for bars in ax.containers:
                ax.bar_label(bars, label_type="center", color='w')

            ax.invert_yaxis()

            parameter_string = ""
            for param in config:
                if param != variable_param and param != "map":
                    parameter_string += f"{param}: {config[param]},\n"
            parameter_string += '\n'

            variable_string = ""
            if variable_param == "agents" or variable_param == "tasks" or variable_param == "pickup" or variable_param == "goal":
                variable_string = f"Number of {variable_param}"
            elif variable_param == "task_freq":
                variable_string = f"Possible task frequencies"
            elif variable_param == "td_update":
                variable_string = f"Task distribution update frequency values"

            ax.set_title(f"Map: {config['map']}\n" + parameter_string,
                                                fontweight='bold', fontsize=self.fontSize, pad=self.padding)
            ax.set_ylabel(variable_string, fontweight='bold', fontsize=self.fontSize)
            ax.set_xlabel(TIME_METRICS[metric_name], fontweight='bold', fontsize=self.fontSize)
            ax.set_yticks(
                [r + barWidth / number_of_bars_per_group for r in range(len(possible_variable_num))],
                possible_variable_num)
            if number_of_bars_per_group > 1:
                ax.legend()

            if metric_name in OTHER_METRICS:
                plt.close(fig)
                continue

            plt.tight_layout()

            plot_type_str = "double_bar_plots/"
            variable_param_value = str(config[variable_param])

            self.save_plot(fig, map_name, metric_name, plot_type_str, variable_param_value)

    def show_metric_evolution(self, map_name: MapName) -> None:
        """
        Show the evolution of some metrics.
        :param map_name: The name of the map to show the metrics of.
        """
        run_ids = self.get_run_ids_from_map(map_name)

        for run_id in run_ids:
            config, df_fixed, df_learning = self.stats_of(run_id=run_id)[0:3]

            for metric_name in TIME_EVOLUTION_NAMES:
                fig, run_ax = plt.subplots(figsize=(500 * self.pixel_size, 500 * self.pixel_size))
                fixed_metric = df_fixed[metric_name]
                learning_metric = df_learning[metric_name]

                if metric_name not in ONLY_LEARNING_TIME_NAMES:
                    run_ax.plot(fixed_metric, label='TP-mp')
                run_ax.plot(learning_metric, label='TP-mp\nwith Estimation')

                if metric_name in MAX_TASK_TIME_NAMES:
                    run_ax.set_xlim(0, config["last_task_time"] + 1)

                parameter_string = ""
                should_new_line = False
                for param in config:
                    if param != "map":
                        parameter_string += f"{param}: {config[param]}, "
                        if should_new_line:
                            parameter_string += '\n'
                        should_new_line = not should_new_line

                run_ax.set_title(f"Map: {config["map"]}\n" + parameter_string, fontweight='bold',
                                               fontsize=self.fontSize, pad=self.padding)
                run_ax.set_xlabel("Time", fontweight='bold', fontsize=self.fontSize)
                run_ax.set_ylabel(TIME_METRICS[metric_name], fontweight='bold', fontsize=self.fontSize)
                if metric_name not in ONLY_LEARNING_TIME_NAMES:
                    run_ax.legend()
                plt.tight_layout()

                plot_type_str = "metric_evolution/"
                variable_param_value = str(config[self.variable_param])

                self.save_plot(fig, map_name, metric_name, plot_type_str, variable_param_value)

    def show_real_vs_estimated_avg_costs(self, map_name: MapName) -> None:
        """
        Show the average real and estimated costs.
        :param map_name: The name of the map to show and compare the estimated and real costs of.
        """
        run_ids = self.get_run_ids_from_map(map_name)

        barWidth = 0.25

        for run_id in run_ids:
            fig, currentAx = plt.subplots(figsize=(500*self.pixel_size, 500*self.pixel_size))

            config, _, _, df_fixed_costs, df_learning_costs, _, _ = self.stats_of(run_id=run_id)
            fixed_estimated_avg = np.mean(df_fixed_costs["estimated"])
            learning_estimated_avg = np.mean(df_learning_costs["estimated"])
            fixed_real_avg = np.mean(df_fixed_costs["real"])
            learning_real_avg = np.mean(df_learning_costs["real"])

            bar0 = np.arange(2)  # fixed estimated/real
            bar1 = [x + barWidth for x in bar0]  # learning estimated/real

            currentAx.bar(bar0, [fixed_estimated_avg, fixed_real_avg], color='r', width=barWidth,
                          edgecolor='grey', label='TP-mp')
            currentAx.bar(bar1, [learning_estimated_avg, learning_real_avg], color='g', width=barWidth,
                          edgecolor='grey', label='TP-mp\nwith Estimation')

            for bars in currentAx.containers:
                currentAx.bar_label(bars, rotation="vertical", label_type="center", color='w')

            should_new_line = False
            parameter_string = ""
            for param in config:
                if param != "map":
                    if should_new_line:
                        parameter_string += '\n'
                    parameter_string += f"{param}: {config[param]}, "
                    should_new_line = not should_new_line

            currentAx.set_title(f"Map: {config["map"]}\n" + parameter_string, fontweight='bold',
                                fontsize=self.fontSize, pad=self.padding)
            currentAx.set_ylabel("Average Cost", fontweight='bold', fontsize=self.fontSize)
            currentAx.set_xticks([r + barWidth / 2 for r in range(2)],
                                 ["Estimated", "Real"])
            currentAx.legend()
            plt.tight_layout()

            plot_type_str = "real_vs_estimated_avg_costs/"
            variable_param_value = str(config[self.variable_param])

            self.save_plot(fig, map_name, "", plot_type_str, variable_param_value)

    def show_traffic_evolution(self, map_name: MapName) -> None:
        """
        Show the evolution of the traffic with a heatmap.
        :param map_name: The name of the map to show the traffic evolution of.
        """
        run_ids = self.get_run_ids_from_map(map_name)

        for run_id in run_ids:
            config, _, _, _, _, traffic_fixed, traffic_learning = self.stats_of(run_id=run_id)
            fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(1000*self.pixel_size, 800*self.pixel_size))

            min_fixed, max_fixed = math.inf, -math.inf

            for i in range(len(traffic_fixed)):
                for j in range(len(traffic_fixed[i])):
                    if traffic_fixed[i][j] < min_fixed:
                        min_fixed = traffic_fixed[i][j]
                    if traffic_fixed[i][j] > max_fixed:
                        max_fixed = traffic_fixed[i][j]

            min_learning, max_learning = math.inf, -math.inf

            for i in range(len(traffic_learning)):
                for j in range(len(traffic_learning[i])):
                    if traffic_learning[i][j] < min_learning:
                        min_learning = traffic_learning[i][j]
                    if traffic_learning[i][j] > max_learning:
                        max_learning = traffic_learning[i][j]

            max_traffic = max(max_fixed, max_learning)
            min_traffic = min(min_fixed, min_learning)

            max_distance_fixed = max(len(dist_list) for dist_list in traffic_fixed)
            max_distance_learning = max(len(dist_list) for dist_list in traffic_learning)

            xticks_fixed = range(1, max_distance_fixed + 1)
            xticks_learning = range(1, max_distance_learning + 1)

            sns.heatmap(traffic_fixed, cmap='YlOrRd', ax=ax[0], vmin=min_traffic, vmax=max_traffic)
            xticks_fixed_locations = ax[0].get_xticks()
            ax[0].set_xticks(xticks_fixed_locations, labels=xticks_fixed)
            ax[0].invert_yaxis()
            ax[0].set_title("TP-mp")
            ax[0].set_xlabel('Distance')
            ax[0].set_ylabel('Time')

            sns.heatmap(traffic_learning, cmap='YlOrRd', ax=ax[1], vmin=min_traffic, vmax=max_traffic)
            xticks_learning_locations = ax[1].get_xticks()
            ax[1].set_xticks(xticks_learning_locations, labels=xticks_learning)
            ax[1].invert_yaxis()
            ax[1].set_title("TP-mp\nwith Estimation")
            ax[1].set_xlabel('Distance')
            ax[1].set_ylabel('Time')

            parameter_string = ""
            for param in config:
                if param != "map":
                    parameter_string += f"{param}: {config[param]},\n"

            fig.suptitle(f"Traffic:\nMap: {config["map"]}\n" + parameter_string, fontweight='bold',
                         fontsize=self.fontSize)

            plt.tight_layout()

            plot_type_str = "traffic_evolution/"
            variable_param_value = str(config[self.variable_param])

            self.save_plot(fig, map_name, "", plot_type_str, variable_param_value)

    def save_plot(self, fig, map_name, metric_name, plot_type_str, variable_param_value):
        results_dir = os.path.join(RootPath.get_root(), 'Utils', 'ShowStats', 'plots_pdfs', self.variable_param,
                                   map_name, plot_type_str)
        if not os.path.isdir(results_dir):
            os.makedirs(results_dir)
        if self.save:
            plt.savefig(results_dir + "_" + metric_name + "_" + variable_param_value + ".pdf")
            plt.close(fig)
        else:
            plt.show()

    def show_all_metrics(self) -> None:
        """
        Show all the metrics of the simulation.
        :param map_name: The name of the map to show the metrics of.
        """

        showed_run_ids = set()

        for my_map in self.maps:
            map_name = my_map["map_name"]
            run_ids = self.get_run_ids_from_map(map_name)

            run_ids = [run_id for run_id in run_ids if run_id not in showed_run_ids]

            if len(run_ids) == 0:
                continue

            self.params["agents"] = my_map["agents_num"]
            self.params["pickup"] = my_map["start_num"]
            self.params["goal"] = my_map["goal_num"]
            self.check_for_variable_param()

            self.show_double_bar_time_metric(map_name)
            self.show_metric_evolution(map_name)
            self.show_real_vs_estimated_avg_costs(map_name)
            self.show_traffic_evolution(map_name)

            showed_run_ids.update(run_ids)
