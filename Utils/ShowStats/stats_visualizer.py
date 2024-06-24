import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

TIME_METRIC_NAMES = ["costs", "serv_times", "pickup_to_goal_times", "start_to_pickup_times", "runtimes", "makespans"]
TIME_METRIC_LABELS = ["Costs per Task", "Service Times", "Pickup to Goal Times", "Start to Pickup Times", "Runtimes",
                      "Makespans"]

TIME_METRICS = dict(zip(TIME_METRIC_NAMES, TIME_METRIC_LABELS))

TIME_EVOLUTION_NAMES = ["serv_times", "pickup_to_goal_times", "start_to_pickup_times", "runtimes"]


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
            "makespans": time["fixed"]
        }, index=time["fixed"])

        df_time_learning = pd.DataFrame({
            "costs": learning["costs"], "serv_times": learning["serv_times"],
            "pickup_to_goal_times": learning["pickup_to_goal_times"],
            "start_to_pickup_times": learning["start_to_pickup_times"], "runtimes": learning["runtimes"],
            "makespans": time["learning"]
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
            "pickup": fixed["pickup"], "goal": fixed["goal"], "task_freq": fixed["task_frequency"]
        }

        return config, df_time_fixed, df_time_learning, df_tasks_fixed, df_tasks_learning

    def get_run_ids_from_map(self, map_name):
        run_ids = []
        for map in self.maps:
            config = self.stats_of(map)[0]
            if config["map"] == map_name:
                run_ids.append(map["run_id"])
        return run_ids

    def show_double_bar_time_metric(self, map_name, metric_name, metric_label):
        run_ids = self.get_run_ids_from_map(map_name)
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 6))

        # set width of bar
        barWidth = 0.25
        possible_variable_num = []
        fixed_metric = []
        learning_metric = []
        variable_param = self.get_variable_config_parameter()

        for run_id in run_ids:
            config, df_fixed, df_learning, df_fixed_costs, df_learning_costs = self.stats_of(run_id=run_id)
            metric = df_fixed[metric_name].iloc[-1]
            fixed_metric.append(metric)
            metric_learning = df_learning[metric_name].iloc[-1]
            learning_metric.append(metric_learning)
            if config[variable_param] not in possible_variable_num:
                possible_variable_num.append(config[variable_param])

        config = self.stats_of(run_id=run_ids[0])[0]

        if metric_name == "costs":
            fixed_metric = [x / config["tasks"] for x in fixed_metric]
            learning_metric = [x / config["tasks"] for x in learning_metric]

        bar0 = np.arange(len(fixed_metric))
        bar1 = [x + barWidth for x in bar0]

        ax.bar(bar0, fixed_metric, color='r', width=barWidth,
               edgecolor='grey', label='Fixed')
        ax.bar(bar1, learning_metric, color='g', width=barWidth,
               edgecolor='grey', label='Learning')

        for bars in ax.containers:
            ax.bar_label(bars)

        parameter_string = ""
        for param in config:
            if param != variable_param and param != "map":
                parameter_string += f"{param}: {config[param]} "
        parameter_string += '\n'
        variable_string = f"Possible values of {variable_param}: {possible_variable_num}"

        plt.xlabel(f"Mappa: {config["map"]}\n" + parameter_string + variable_string, fontweight='bold', fontsize=8)
        plt.ylabel(metric_label, fontweight='bold', fontsize=8)
        plt.xticks([r + barWidth / 2 for r in range(len(possible_variable_num))],
                   possible_variable_num)
        plt.legend()
        plt.show()

    def show_metric_evolution(self, map_name, metric_name, metric_label):
        run_ids = self.get_run_ids_from_map(map_name)

        for run_id in run_ids:
            config, df_fixed, df_learning, df_fixed_costs, df_learning_costs = self.stats_of(run_id=run_id)
            fixed_metric = df_fixed[metric_name]
            learning_metric = df_learning[metric_name]

            fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 6))
            ax.plot(fixed_metric, label='Fixed')
            ax.plot(learning_metric, label='Learning')

            parameter_string = ""
            for param in config:
                if param != "map":
                    parameter_string += f"{param}: {config[param]} "

            plt.xlabel(f"Mappa: {config["map"]}\n" + parameter_string, fontweight='bold', fontsize=8)
            plt.ylabel(metric_label, fontweight='bold', fontsize=8)
            plt.legend()
            plt.show()

    def show_real_vs_estimated_avg_costs(self, map_name):
        run_ids = self.get_run_ids_from_map(map_name)

        barWidth = 0.25

        for run_id in run_ids:
            fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
            config, df_fixed, df_learning, df_fixed_costs, df_learning_costs = self.stats_of(run_id=run_id)
            fixed_estimated_avg = np.mean(df_fixed_costs["estimated"])
            learning_estimated_avg = np.mean(df_learning_costs["estimated"])
            fixed_real_avg = np.mean(df_fixed_costs["real"])
            learning_real_avg = np.mean(df_learning_costs["real"])

            bar0 = np.arange(2)  # fixed estimated/real
            bar1 = [x + barWidth for x in bar0]  # learning estimated/real

            ax.bar(bar0, [fixed_estimated_avg, fixed_real_avg], color='r', width=barWidth,
                   edgecolor='grey', label='Fixed')
            ax.bar(bar1, [learning_estimated_avg, learning_real_avg], color='g', width=barWidth,
                   edgecolor='grey', label='Learning')

            for bars in ax.containers:
                ax.bar_label(bars)

            parameter_string = ""
            for param in config:
                if param != "map":
                    parameter_string += f"{param}: {config[param]} "

            plt.xlabel(f"Mappa: {config["map"]}\n" + parameter_string, fontweight='bold', fontsize=8)
            plt.ylabel("Average Cost", fontweight='bold', fontsize=8)
            plt.xticks([r + barWidth / 2 for r in range(2)],
                       ["Estimated", "Real"])
            plt.legend()
            plt.show()
