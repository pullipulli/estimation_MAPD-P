"""
This script generates results for the given maps, agents, starts, goals, tasks and task frequencies.
It generates results for both fixed and learning cases.
author: Andrea Pullia (@pullipulli)
"""

import datetime
import json
from collections import defaultdict
from glob import glob
from statistics import *
from argparse import ArgumentParser, ArgumentTypeError
import time

import yaml
import RootPath
import os.path
import numpy as np
import random

from Simulation.TP_with_recovery import TokenPassingRecovery
from Simulation.simulation_new_recovery import SimulationNewRecovery
from Utils.type_checking import RunId, MapOutput, StatSimulation, StatJson, MapStats
from typing import Set

from termcolor import colored
from icecream import ic as print


class GenerateResults:
    """
    This class generates results for the given maps, agents, starts, goals, tasks, task frequencies and task distribution update number.
    """

    def __init__(self, maps: list[MapStats], tasks_num: list[int], tasks_frequency: list[float], task_distr_update_num: list[int], max_distance_traffic: int):
        self.simulation_number = len(maps) * len(tasks_num) * len(agents_num) * len(start_num) * len(
            goal_num * len(tasks_frequency))  * 2 * len(task_distr_update_num)
        self.maps = maps
        self.tasks_num = tasks_num
        self.tasks_frequency = tasks_frequency
        self.task_distr_update_num = task_distr_update_num
        self.max_distance_traffic = max_distance_traffic
        self.simulation_progress = 0
        self.maps_out: list[MapOutput] = []
        self.run_ids: Set[RunId] = set()


    def generate_results(self) -> None:
        """
        Generates the results for the given maps, agents, starts, goals, tasks and task frequencies.
        Writes the results in a json file (saved in the ResultsJsons folder). The file name is the current date and time.
        """

        def uniquify(path: str):
            """Returns a unique path by adding a number to the end of the path if it already exists."""
            filename, extension = os.path.splitext(path)
            counter = 1

            while os.path.exists(path):
                path = filename + " (" + str(counter) + ")" + extension
                counter += 1

            return path

        for myMap in self.maps:
            self.generate_output_map(myMap)

        output: StatJson = {"maps": self.maps_out, "tasks_num": self.tasks_num, "tasks_frequency": self.tasks_frequency,
                            "task_distr_update_num": self.task_distr_update_num }

        timestr = time.strftime("%d_%m_%Y__%H_%M_%S")

        jsonFileName = uniquify(RootPath.get_root() + '/Utils/ShowStats/ResultsJsons/results_' + timestr + '.json')

        with open(jsonFileName, 'w+') as f:
            json.dump(output, f, separators=(',', ':'))

        print(colored(jsonFileName, "cyan"))

    @staticmethod
    def memorize_run_stats(old_stats: StatSimulation, start_to_goal_times: list[float],
                           start_to_pickup_times: list[float], pickup_to_goal_times: list[float],
                           actual_runtime: float, running_simulation: SimulationNewRecovery,
                           token_passing: TokenPassingRecovery) -> StatSimulation:
        """
        Memorizes the statistics of the current run.
        It calculates the average start_to_goal_times, start_to_pickup_times, pickup_to_goal_times, serv_times, runtimes,
        costs and earth_mover_distance.
        :param old_stats:
        :param start_to_goal_times:
        :param start_to_pickup_times:
        :param pickup_to_goal_times:
        :param actual_runtime:
        :param running_simulation:
        :param token_passing:
        :return:
        """
        if running_simulation.time == 1:
            return {"costs": [0], "serv_times": [0], "start_to_pickup_times": [0], "pickup_to_goal_times": [0],
                    "runtimes": [actual_runtime], "earth_mover_dist": [running_simulation.get_earth_mover_distance()]}
        else:
            pickup_to_goal_avgs = old_stats["pickup_to_goal_times"]
            start_to_pickup_avgs = old_stats["start_to_pickup_times"]
            serv_times = old_stats["serv_times"]
            runtimes = old_stats["runtimes"]
            costs = old_stats["costs"]
            earth_mover_distance = old_stats["earth_mover_dist"]

            cost = 0
            for path in running_simulation.actual_paths.values():
                cost += len(path)

            completed_tasks_names = list(token_passing.get_completed_tasks_times().keys())

            pick_up_tasks_names = list(token_passing.get_pick_up_tasks_times().keys())

            for task_name in pick_up_tasks_names:
                if task_name == "test" or task_name == "safe_idle":
                    pick_up_tasks_names.remove(task_name)

            # Calculate and memorize the time between start and goal of the last task
            if len(completed_tasks_names) != 0:
                last_completed_task_time = token_passing.get_completed_tasks_times()[completed_tasks_names[-1]]
                start_time_of_last_task = token_passing.get_start_tasks_times()[completed_tasks_names[-1]]
                start_to_goal_time = last_completed_task_time - start_time_of_last_task
                start_to_goal_times.append(start_to_goal_time)
            else:
                start_to_goal_times.append(0)

            # Calculate and memorize the time between start and pickup of the last task
            if len(pick_up_tasks_names) != 0:
                last_pickup_time = token_passing.get_pick_up_tasks_times()[pick_up_tasks_names[-1]]
                start_time_of_pickup = token_passing.get_start_tasks_times()[pick_up_tasks_names[-1]]
                start_to_pickup_time = last_pickup_time - start_time_of_pickup
                start_to_pickup_times.append(start_to_pickup_time)
            else:
                start_to_pickup_times.append(0)

            # Calculate and memorize the time between pickup and goal of the last task
            if len(completed_tasks_names) != 0:
                last_completed_task_time = token_passing.get_completed_tasks_times()[completed_tasks_names[-1]]
                pickup_time_of_completed = token_passing.get_pick_up_tasks_times()[completed_tasks_names[-1]]
                pickup_to_goal_time = last_completed_task_time - pickup_time_of_completed
                pickup_to_goal_times.append(pickup_to_goal_time)
            else:
                pickup_to_goal_times.append(0)

            start_to_pickup_avgs.append(mean(start_to_pickup_times))
            pickup_to_goal_avgs.append(mean(pickup_to_goal_times))
            serv_times.append(mean(start_to_goal_times))
            runtimes.append(actual_runtime)
            costs.append(cost)
            earth_mover_distance.append(running_simulation.get_earth_mover_distance())

            return {"costs": costs, "serv_times": serv_times, "start_to_pickup_times": start_to_pickup_avgs,
                    "pickup_to_goal_times": pickup_to_goal_avgs, "runtimes": runtimes,
                    "earth_mover_dist": earth_mover_distance}

    def generate_output_map(self, map: MapStats):
        """
        Generates the output for the given map. It generates results for both fixed and learning cases.
        :param map:
        :return:
        """
        for agents in map['agents_num']:
            for starts in map['start_num']:
                for goals in map['goal_num']:
                    for tasks in self.tasks_num:
                        for task_frequency in self.tasks_frequency:
                            for task_distr_update in self.task_distr_update_num:
                                result_fixed = self.simulate(map, map["name"], agents, starts, goals, tasks,
                                                             task_distr_update,
                                                             task_frequency, learning=False)
                                self.simulation_progress += 1
                                print(f"Progress: {(self.simulation_progress / self.simulation_number):.2%}")

                                results_learning = self.simulate(map, map["name"], agents, starts, goals, tasks,
                                                                 task_distr_update,
                                                                 task_frequency, learning=True)
                                self.simulation_progress += 1
                                print(f"Progress: {(self.simulation_progress / self.simulation_number):.2%}")

                                run_id = (str(result_fixed['map_name']) + "_agents_" + str(
                                    result_fixed['agents']) + "_pickup_" +
                                          str(result_fixed['pickup']) + "_goal_" + str(result_fixed['goal']) +
                                          "_tasks_" + str(result_fixed['tasks']) + "_task_frequency_" + str(
                                            result_fixed['task_frequency']) + "_task_distr_update_" + str(
                                            result_fixed['task_distr_update']))

                                if run_id not in self.run_ids:
                                    self.maps_out.append(
                                        {'run_id': run_id, 'fixed': result_fixed, 'learning': results_learning, 'agents_num': map['agents_num'],
                                         'start_num': map['start_num'], 'goal_num': map['goal_num'], 'map_name': map['name']})
                                    self.run_ids.add(run_id)

    def check_collisions(self, simulation: SimulationNewRecovery):
        """
        Checks the number of path and switch collisions in the simulation.
        :param simulation:
        :return:
        """
        pathCollisions = 0
        switchCollisions = 0
        for agent in simulation.actual_paths.keys():
            for t in range(0, simulation.get_time() - 1):
                for agent2 in simulation.actual_paths.keys():
                    if agent2 != agent:
                        if simulation.actual_paths[agent][t] == simulation.actual_paths[agent2][t]:
                            pathCollisions += 1
                        if simulation.actual_paths[agent][t] == simulation.actual_paths[agent2][t + 1] and \
                                simulation.actual_paths[agent][t + 1] == simulation.actual_paths[agent2][t]:
                            switchCollisions += 1
        print(f"Path collisions: {pathCollisions}")
        print(f"Switch collisions: {switchCollisions}")

    def simulate(self, map_dict: MapStats, map_name: str, agents_num: int, starts_num: int, goals_num: int,
                 tasks_num: int, task_distr_num, tasks_frequency: float,
                 learning=False) -> StatSimulation:
        """
        Simulates the given map with the given agents, starts, goals, tasks and task frequency. (fixed or learning case)
        :param task_distr_num:
        :param map_dict:
        :param map_name:
        :param agents_num:
        :param starts_num:
        :param goals_num:
        :param tasks_num:
        :param tasks_frequency:
        :param learning:
        :return:
        """
        goal_locations = map_dict['map']['goal_locations']
        start_locations = map_dict['map']['start_locations']

        total_goals = len(goal_locations)
        total_starts = len(start_locations)

        goals_num = min(goals_num, len(map_dict['map']['goal_locations']))
        starts_num = min(starts_num, len(map_dict['map']['start_locations']))

        # removing random n start from the list of start locations
        starts_to_delete = set(random.sample(range(total_starts), total_starts - starts_num))
        start_locations = [start for startIndex, start in enumerate(start_locations) if
                           startIndex not in starts_to_delete]

        # removing random n goals from the list of goal locations
        goals_to_delete = set(random.sample(range(total_goals), total_goals - goals_num))
        goal_locations = [goal for goalIndex, goal in enumerate(goal_locations) if goalIndex not in goals_to_delete]

        dimensions = map_dict['map']['dimensions']
        max_time = 100000
        task_distributions = [dict() for i in range(max_time)]
        tasks = []
        total = 0
        time = 0
        last_task_time = max_time
        while total < tasks_num:
            task_distribution = dict()
            tasks_now = np.random.poisson(tasks_frequency)
            locations = []
            if tasks_now > tasks_num - total:
                tasks_now = tasks_num - total
            while tasks_now > 0:
                tasks_now -= 1
                locations.append(random.choice(start_locations))
            for start in start_locations:
                if start in locations:
                    probability = 1.0
                    task_distribution[tuple(start)] = probability
                    total += 1
                    goal = random.choice(goal_locations)
                    tasks.append(
                        {'start_time': time, 'start': start, 'goal': goal,
                         'task_name': 'task' + str(total)})
                    last_task_time = time
            task_distributions[time] = task_distribution
            time += 1

        dimensions = map_dict['map']['dimensions']
        obstacles = map_dict['map']['obstacles']
        non_task_endpoints = map_dict['map']['non_task_endpoints']
        agents = map_dict['agents']
        map_dict['tasks'] = tasks

        total_agents = len(agents)
        agents_num = min(agents_num, total_agents)

        # removing random n agents from the list of agents
        agents_to_delete = set(random.sample(range(total_agents), total_agents - agents_num))
        agents = [agent for agentIndex, agent in enumerate(agents) if agentIndex not in agents_to_delete]

        simulation = SimulationNewRecovery(tasks, agents, task_distributions, learning, task_distr_num, last_task_time,
                                           max_time,
                                           max_distance_traffic=self.max_distance_traffic)
        tp = TokenPassingRecovery(agents, dimensions, max_time, obstacles, non_task_endpoints, simulation,
                                  start_locations,
                                  goal_locations,
                                  a_star_max_iter=800000000, path_1_modified=True,
                                  path_2_modified=True,
                                  preemption_radius=3,
                                  preemption_duration=3)

        runtime = 0
        stats = defaultdict(lambda: [])
        start_to_goal_times = []
        start_to_pickup_times = []
        pickup_to_goal_times = []

        print(f"Avvio Simulazione:"
              f"\n\tNome Mappa: {map_name}"
              f"\n\tNumero Agenti: {len(agents)}"
              f"\n\tNumero pickup: {len(start_locations)}"
              f"\n\tNumero goal: {len(goal_locations)}"
              f"\n\tNumero task: {tasks_num}"
              f"\n\tTask frequency: {tasks_frequency:.2f}"
              f"\n\tTask Distribution Update Frequency: {task_distr_num}"
              f"\n\tLearning: {learning}")

        while tp.get_completed_tasks() != len(tasks):
            initialTime = datetime.datetime.now().timestamp()

            simulation.time_forward(tp)

            final = datetime.datetime.now().timestamp()
            runtime += final - initialTime

            stats: StatSimulation = GenerateResults.memorize_run_stats(stats, start_to_goal_times,
                                                                       start_to_pickup_times, pickup_to_goal_times,
                                                                       runtime,
                                                                       simulation, tp)

        self.check_collisions(simulation)
        print(f"Deadlocks: {tp.get_token()['deadlock_count_per_agent']}")

        print("Simulation finished")

        stats['traffic'] = simulation.traffic_matrix
        stats['agents'] = len(agents)
        stats['pickup'] = len(start_locations)
        stats['goal'] = len(goal_locations)
        stats['tasks'] = tasks_num
        stats['task_frequency'] = tasks_frequency
        stats['task_distr_update'] = task_distr_num
        stats['estimated_costs'] = tp.get_estimated_task_costs()
        stats['real_costs'] = tp.get_real_task_costs()
        stats['map_name'] = map_name
        stats['last_task_time'] = last_task_time

        return stats


if __name__ == '__main__':
    def positive_integer(value: str):
        """
        Check if the value is a positive integer. If not, raises an ArgumentTypeError.
        """
        ivalue = int(value)
        if ivalue < 1:
            raise ArgumentTypeError("%s is an invalid positive int value" % value)
        return ivalue

    parser = ArgumentParser()

    parser.add_argument('-max_distance_traffic', default=5, type=positive_integer,
                        help='Max distance to consider when calculating traffic matrix')
    agent_group = parser.add_mutually_exclusive_group()
    agent_group.add_argument('-agents', default=1, type=positive_integer,
                             help='Number of possible agent combinations')
    agent_group.add_argument('-agents_list', nargs='+', type=positive_integer,
                             help='List of agent combinations')

    start_group = parser.add_mutually_exclusive_group()
    start_group.add_argument('-starts', default=1, type=positive_integer,
                             help='Number of possible starts combinations')
    start_group.add_argument('-starts_list', nargs='+', type=positive_integer,
                             help='List of start combinations')

    goal_group = parser.add_mutually_exclusive_group()
    goal_group.add_argument('-goals', default=1, type=positive_integer,
                            help='Number of possible goals combinations')
    goal_group.add_argument('-goals_list', nargs='+', type=positive_integer,
                            help='List of goal combinations')

    task_freq_group = parser.add_mutually_exclusive_group()
    task_freq_group.add_argument('-tasks_frequency', default=1, type=positive_integer,
                                 help='Number of possible task_frequencies combinations')
    task_freq_group.add_argument('-tasks_frequency_list', nargs='+', type=float,
                                 help='List of task_frequencies combinations')

    task_group = parser.add_mutually_exclusive_group()
    task_group.add_argument('-tasks', default=1, type=positive_integer,
                            help='Number of possible task combinations')
    task_group.add_argument('-tasks_list', nargs='+', type=positive_integer,
                            help='List of task combinations')

    task_distribution_update_group = parser.add_mutually_exclusive_group()
    task_distribution_update_group.add_argument('-td_update', default=1, type=positive_integer,
                                                help='Number of possible task distribution update values')
    task_distribution_update_group.add_argument('-td_update_list', nargs='+', type=positive_integer,
                                                help='List of possible task distribution update values')
    args = parser.parse_args()

    print(args)

    map_file_names = glob(os.path.join(RootPath.get_root(), 'Environments', '*.yaml'))

    maps: list[MapStats] = []

    max_tasks_frequency = 1
    max_tasks = 100

    if args.tasks_list:
        tasks_num = args.tasks_list
    elif args.tasks == 1:
        tasks_num = [max_tasks]
    else:
        tasks_num = np.linspace(5, max_tasks, args.tasks, dtype=int).tolist()

    if args.tasks_frequency_list:
        tasks_frequency = args.tasks_frequency_list
    elif args.tasks_frequency == 1:
        tasks_frequency = [max_tasks_frequency]
    else:
        tasks_frequency = np.linspace(0.05, max_tasks_frequency, args.tasks_frequency).tolist()

    if args.td_update_list:
        td_update_num = args.td_update_list
    elif args.td_update == 1:
        td_update_num = [15]
    else:
        td_update_num = np.linspace(1, 50, args.td_update, dtype=int).tolist()

    for map_file_name in map_file_names:
        with open(map_file_name, 'r') as map_file:
            try:
                map_yaml = yaml.load(map_file, Loader=yaml.FullLoader)
                max_agents = len(map_yaml['agents'])
                max_starts = len(map_yaml['map']['start_locations'])
                max_goals = len(map_yaml['map']['goal_locations'])

                if args.agents_list:
                    agents_num = args.agents_list
                elif args.agents == 1:
                    agents_num = [max_agents]
                else:
                    agents_num = np.linspace(3, max_agents, args.agents, dtype=int).tolist()

                if args.starts_list:
                    start_num = args.starts_list
                elif args.starts == 1:
                    start_num = [max_starts]
                else:
                    start_num = np.linspace(15, max_starts, args.starts, dtype=int).tolist()

                if args.goals_list:
                    goal_num = args.goals_list
                elif args.goals == 1:
                    goal_num = [max_goals]
                else:
                    goal_num = np.linspace(15, max_goals, args.goals, dtype=int).tolist()

                map_yaml['agents_num'] = agents_num
                map_yaml['start_num'] = start_num
                map_yaml['goal_num'] = goal_num
                map_yaml['name'] = map_file_name.split('/')[-1].split('.')[0]

                maps.append(map_yaml)
            except yaml.YAMLError as exc:
                print(exc)

    gen_result = GenerateResults(maps, tasks_num, tasks_frequency, td_update_num, args.max_distance_traffic)

    gen_result.generate_results()
