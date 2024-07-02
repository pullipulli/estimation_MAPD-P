from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
import seaborn as sns

TIME_METRIC_NAMES = ["costs", "serv_times", "pickup_to_goal_times", "start_to_pickup_times", "runtimes", "makespans",
                     "earth_mover_dist"]
TIME_METRIC_LABELS = ["Costs per Task", "Service Times", "Pickup to Goal Times", "Start to Pickup Times", "Runtimes",
                      "Makespans", "Earth Mover Distance"]
MAX_TASK_TIME_NAMES = ["earth_mover_dist"]
ONLY_LEARNING_TIME_NAMES = ["earth_mover_dist"]

TIME_METRICS = dict(zip(TIME_METRIC_NAMES, TIME_METRIC_LABELS))

TIME_EVOLUTION_NAMES = ["serv_times", "pickup_to_goal_times", "start_to_pickup_times", "runtimes", "earth_mover_dist"]


class StatsVisualizer:
    def __init__(self, maps, agents_num, tasks_num, task_frequency_num, pickup_num, goal_num):
        self.maps = maps
        self.map_names = set()
        for map in self.maps:
            config = self.stats_of(map)[0]
            self.map_names.add(config["map"])

        self.params = {
            "agents": agents_num,
            "tasks": tasks_num,
            "task_frequency": task_frequency_num,
            "pickup": pickup_num,
            "goal": goal_num
        }
        variable_params = []
        for param in self.params:
            if len(self.params[param]) > 1:
                variable_params.append(param)
        assert len(variable_params) == 1, "Only one parameter can be variable"
        self.variable_param = variable_params[0]

        self.padding = 0
        self.fontSize = 12

    def get_map_names(self):
        return self.map_names

    def get_variable_config_parameter(self):
        return self.variable_param

    def stats_of(self, map_dict=None, run_id=None):
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
               fixed['goal'] == learning['goal']

        config = {
            "agents": fixed["agents"], "tasks": fixed["tasks"], "map": fixed["map_name"],
            "pickup": fixed["pickup"], "goal": fixed["goal"], "task_frequency": fixed["task_frequency"],
            "last_task_time": max(fixed["last_task_time"], learning["last_task_time"])
        }

        return config, df_time_fixed, df_time_learning, df_tasks_fixed, df_tasks_learning, fixed["traffic"], learning["traffic"]

    def get_run_ids_from_map(self, map_name):
        run_ids = []
        for map in self.maps:
            config = self.stats_of(map)[0]
            if config["map"] == map_name:
                run_ids.append(map["run_id"])
        return run_ids

    def show_double_bar_time_metric(self, map_name, ax, row_number=2):
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

        rowIndex = 0
        columnIndex = 0
        metric_index = 0
        for metric_name in TIME_METRIC_NAMES:
            number_of_bars_per_group = 2
            if metric_index == math.ceil(len(TIME_METRIC_NAMES) / row_number):
                rowIndex += 1
                columnIndex = 0

            if metric_name == "costs":
                fixed_metric[metric_name] = [x / config["tasks"] for x in fixed_metric[metric_name]]
                learning_metric[metric_name] = [x / config["tasks"] for x in learning_metric[metric_name]]

            bar0 = np.arange(len(fixed_metric[metric_name]))
            bar1 = [x + barWidth for x in bar0]

            if metric_name in ONLY_LEARNING_TIME_NAMES:
                number_of_bars_per_group = 1

            if metric_name not in ONLY_LEARNING_TIME_NAMES:
                ax[rowIndex][columnIndex].barh(bar0, fixed_metric[metric_name], color='r', height=barWidth,
                                               edgecolor='grey', label='Fixed')
            ax[rowIndex][columnIndex].barh(bar1, learning_metric[metric_name], color='g', height=barWidth,
                                           edgecolor='grey', label='Learning')

            for bars in ax[rowIndex][columnIndex].containers:
                ax[rowIndex][columnIndex].bar_label(bars, label_type="center", color='w')

            ax[rowIndex][columnIndex].invert_yaxis()

            parameter_string = ""
            for param in config:
                if param != variable_param and param != "map":
                    parameter_string += f"{param}: {config[param]}, "
            parameter_string += '\n'
            variable_string = f"Possible values of {variable_param}"

            ax[rowIndex][columnIndex].set_title(f"Mappa: {config["map"]}\n" + parameter_string,
                                                fontweight='bold', fontsize=self.fontSize, pad=self.padding)
            ax[rowIndex][columnIndex].set_ylabel(variable_string, fontweight='bold', fontsize=self.fontSize)
            ax[rowIndex][columnIndex].set_xlabel(TIME_METRICS[metric_name], fontweight='bold', fontsize=self.fontSize)
            ax[rowIndex][columnIndex].set_yticks(
                [r + barWidth / number_of_bars_per_group for r in range(len(possible_variable_num))],
                possible_variable_num)
            if number_of_bars_per_group > 1:
                ax[rowIndex][columnIndex].legend()
            metric_index += 1
            columnIndex += 1
        plt.tight_layout()
        plt.show()

    def show_metric_evolution(self, map_name, ax):
        run_ids = self.get_run_ids_from_map(map_name)

        run_index = 0

        for run_id in run_ids:
            config, df_fixed, df_learning = self.stats_of(run_id=run_id)[0:3]

            run_ax = ax

            if len(run_ids) > 1:
                run_ax = ax[run_index]

            metric_index = 0

            for metric_name in TIME_EVOLUTION_NAMES:
                fixed_metric = df_fixed[metric_name]
                learning_metric = df_learning[metric_name]

                if metric_name not in ONLY_LEARNING_TIME_NAMES:
                    run_ax[metric_index].plot(fixed_metric, label='Fixed')
                run_ax[metric_index].plot(learning_metric, label='Learning')

                if metric_name in MAX_TASK_TIME_NAMES:
                    run_ax[metric_index].set_xlim(0, config["last_task_time"] + 1)

                parameter_string = ""
                should_new_line = False
                for param in config:
                    if param != "map":
                        parameter_string += f"{param}: {config[param]}, "
                        if should_new_line:
                            parameter_string += '\n'
                        should_new_line = not should_new_line

                run_ax[metric_index].set_title(f"Mappa: {config["map"]}\n" + parameter_string, fontweight='bold',
                                               fontsize=self.fontSize, pad=self.padding)
                run_ax[metric_index].set_xlabel("Time", fontweight='bold', fontsize=self.fontSize)
                run_ax[metric_index].set_ylabel(TIME_METRICS[metric_name], fontweight='bold', fontsize=self.fontSize)
                run_ax[metric_index].legend()
                metric_index += 1
            run_index += 1
        plt.tight_layout()
        plt.show()

    def show_real_vs_estimated_avg_costs(self, map_name, ax):
        run_ids = self.get_run_ids_from_map(map_name)

        barWidth = 0.25

        i = 0
        currentAx = ax
        for run_id in run_ids:
            if len(run_ids) > 1:
                currentAx = ax[i]
            config, _, _, df_fixed_costs, df_learning_costs, _, _ = self.stats_of(run_id=run_id)
            fixed_estimated_avg = np.mean(df_fixed_costs["estimated"])
            learning_estimated_avg = np.mean(df_learning_costs["estimated"])
            fixed_real_avg = np.mean(df_fixed_costs["real"])
            learning_real_avg = np.mean(df_learning_costs["real"])

            bar0 = np.arange(2)  # fixed estimated/real
            bar1 = [x + barWidth for x in bar0]  # learning estimated/real

            currentAx.bar(bar0, [fixed_estimated_avg, fixed_real_avg], color='r', width=barWidth,
                          edgecolor='grey', label='Fixed')
            currentAx.bar(bar1, [learning_estimated_avg, learning_real_avg], color='g', width=barWidth,
                          edgecolor='grey', label='Learning')

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

            currentAx.set_title(f"Mappa: {config["map"]}\n" + parameter_string, fontweight='bold',
                                fontsize=self.fontSize, pad=self.padding)
            currentAx.set_ylabel("Average Cost", fontweight='bold', fontsize=self.fontSize)
            currentAx.set_xticks([r + barWidth / 2 for r in range(2)],
                                 ["Estimated", "Real"])
            currentAx.legend()
            i += 1
        plt.tight_layout()
        plt.show()

    def show_traffic_evolution(self, map_name):
        run_ids = self.get_run_ids_from_map(map_name)

        for run_id in run_ids:
            config, _, _, _, _, traffic_fixed, traffic_learning = self.stats_of(run_id=run_id)
            fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(14, 7))

            max_distance_fixed = max(len(dist_list) for dist_list in traffic_fixed)
            max_distance_learning = max(len(dist_list) for dist_list in traffic_learning)

            xticks_fixed = range(1, max_distance_fixed + 1)
            xticks_learning = range(1, max_distance_learning + 1)

            sns.heatmap(traffic_fixed, cmap='YlOrRd', ax=ax[0])
            xticks_fixed_locations = ax[0].get_xticks()
            ax[0].set_xticks(xticks_fixed_locations, labels=xticks_fixed)
            ax[0].invert_yaxis()
            ax[0].set_title("Learning")
            ax[0].set_xlabel('Distance')
            ax[0].set_ylabel('Time')

            sns.heatmap(traffic_learning, cmap='YlOrRd', ax=ax[1])
            xticks_learning_locations = ax[1].get_xticks()
            ax[1].set_xticks(xticks_learning_locations, labels=xticks_learning)
            ax[1].invert_yaxis()
            ax[1].set_title("Learning")
            ax[1].set_xlabel('Distance')
            ax[1].set_ylabel('Time')

            parameter_string = ""
            for param in config:
                if param != "map":
                    parameter_string += f"{param}: {config[param]}, "

            fig.suptitle(f"Traffic:\nMappa: {config["map"]}\n" + parameter_string, fontweight='bold',
                                fontsize=self.fontSize)

            plt.tight_layout()
            plt.show()

    def show_all_metrics(self, map_name):
        width_coefficient = 7
        height_coefficient = 7
        run_ids = self.get_run_ids_from_map(map_name)
        double_bar_rows = 2

        fig, ax = plt.subplots(nrows=double_bar_rows, ncols=math.ceil(len(TIME_METRIC_NAMES) / double_bar_rows),
                               figsize=(width_coefficient * math.ceil(len(TIME_METRIC_NAMES) / double_bar_rows),
                                        height_coefficient * double_bar_rows))
        ax[-1][-1].axis('off')
        self.show_double_bar_time_metric(map_name, ax, row_number=double_bar_rows)

        fig, ax = plt.subplots(nrows=len(run_ids), ncols=len(TIME_EVOLUTION_NAMES), figsize=(
            width_coefficient * len(TIME_EVOLUTION_NAMES), height_coefficient * len(run_ids)))
        self.show_metric_evolution(map_name, ax)

        fig, ax = plt.subplots(nrows=1, ncols=len(run_ids),
                               figsize=(width_coefficient * len(run_ids), height_coefficient))
        self.show_real_vs_estimated_avg_costs(map_name, ax)

        self.show_traffic_evolution(map_name)
