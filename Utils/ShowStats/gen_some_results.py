import datetime
import json
import sys
from collections import defaultdict
from glob import glob
from statistics import *
from argparse import ArgumentParser, ArgumentTypeError
import time

import yaml
import RootPath
import os
import numpy as np
import random

from Simulation.TP_with_recovery import TokenPassingRecovery
from Simulation.simulation_new_recovery import SimulationNewRecovery


class GenerateResults:
    def __init__(self, maps, tasks_num, tasks_frequency, agents_num, start_num, goal_num):
        self.simulation_number = len(maps) * len(tasks_num) * len(agents_num) * len(start_num) * len(
            goal_num * len(tasks_frequency)) * 2
        self.maps = maps
        self.tasks_num = tasks_num
        self.tasks_frequency = tasks_frequency
        self.agents_num = agents_num
        self.start_num = start_num
        self.goal_num = goal_num
        self.simulation_progress = 0
        self.maps_out = []
        self.run_ids = set()

    def generate_results(self):
        for myMap in maps:
            self.generate_output_map(myMap)

        output = {"maps": self.maps_out, "tasks_num": self.tasks_num, "tasks_frequency": self.tasks_frequency,
                  "agents_num": self.agents_num,
                  "start_num": self.start_num, "goal_num": self.goal_num}
        timestr = time.strftime("%d_%m_%Y__%H_%M_%S")

        with open(RootPath.get_root() + '/Utils/ShowStats/ResultsJsons/results_' + timestr + '.json', 'w+') as f:
            json.dump(output, f, separators=(',', ':'))

        print("Results saved in: ", 'results_' + timestr + '.json')

    @staticmethod
    def memorize_run_stats(old_stats, start_to_goal_times, start_to_pickup_times, pickup_to_goal_times,
                           actual_runtime: float, running_simulation: SimulationNewRecovery,
                           token_passing: TokenPassingRecovery):
        if running_simulation.time == 1:
            return {"costs": [0], "serv_times": [0], "start_to_pickup_times": [0], "pickup_to_goal_times": [0],
                    "runtimes": [actual_runtime], "earth_mover_dist": [running_simulation.get_earth_mover_distance()], "number_of_tasks": len(running_simulation.tasks)}
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
                    "pickup_to_goal_times": pickup_to_goal_avgs, "runtimes": runtimes, "earth_mover_dist": earth_mover_distance,
                    "number_of_tasks": len(running_simulation.tasks)}

    def generate_output_map(self, map):
        for agents in self.agents_num:
            for starts in self.start_num:
                for goals in self.goal_num:
                    for tasks in self.tasks_num:
                        for task_frequency in self.tasks_frequency:
                            result_fixed = self.simulate(map, map["name"], agents, starts, goals, tasks, task_frequency, learning=False)
                            self.simulation_progress += 1
                            print("Progress: ", format(self.simulation_progress / self.simulation_number, ".2%"))

                            results_learning = self.simulate(map, map["name"], agents, starts, goals, tasks, task_frequency, learning=True)
                            self.simulation_progress += 1
                            print("Progress: ", format(self.simulation_progress / self.simulation_number, ".2%"))

                            run_id = (str(result_fixed['map_name']) + "_agents_" + str(
                                result_fixed['agents']) + "_pickup_" +
                                      str(result_fixed['pickup']) + "_goal_" + str(result_fixed['goal']) +
                                      "_tasks_" + str(result_fixed['tasks']) + "_task_frequency_" + str(result_fixed['task_frequency']))

                            if run_id not in self.run_ids:
                                self.maps_out.append({'run_id': run_id, 'fixed': result_fixed, 'learning': results_learning})
                                self.run_ids.add(run_id)

    @staticmethod
    def check_collisions(simulation):
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
        print("Path collisions: ", pathCollisions)
        print("Switch collisions: ", switchCollisions)

    @staticmethod
    def simulate(map_dict, map_name, agents_num, starts_num, goals_num, tasks_num, tasks_frequency, learning=False):
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
        max_time = 10000
        dimensions = (dimensions[0], dimensions[1], max_time)
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

        np.set_printoptions(threshold=sys.maxsize)

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

        print("Running Simulation...")

        simulation = SimulationNewRecovery(tasks, agents, task_distributions, learning, 15, last_task_time)
        tp = TokenPassingRecovery(agents, dimensions, max_time, obstacles, non_task_endpoints, simulation,
                                  start_locations,
                                  a_star_max_iter=80000, path_1_modified=True,
                                  path_2_modified=True,
                                  preemption_radius=3,
                                  preemption_duration=3)

        runtime = 0
        stats = defaultdict(lambda: [])
        start_to_goal_times = []
        start_to_pickup_times = []
        pickup_to_goal_times = []

        while tp.get_completed_tasks() != len(tasks):
            initialTime = datetime.datetime.now().timestamp()

            simulation.time_forward(tp)

            final = datetime.datetime.now().timestamp()
            runtime += final - initialTime

            stats = GenerateResults.memorize_run_stats(stats, start_to_goal_times, start_to_pickup_times, pickup_to_goal_times, runtime,
                                       simulation, tp)

        print('Avvio Simulazione:', "\n\tNome Mappa:", map_name, "\n\tNumero Agenti:", len(agents),
              "\n\tNumero pickup:", len(start_locations), "\n\tNumero goal:", len(goal_locations),
              "\n\tNumero task:", tasks_num, "\n\tTask frequency:", tasks_frequency, "\n\tLearning:", learning)
        GenerateResults.check_collisions(simulation)

        stats['agents'] = len(agents)
        stats['pickup'] = len(start_locations)
        stats['goal'] = len(goal_locations)
        stats['tasks'] = tasks_num
        stats['task_frequency'] = tasks_frequency
        stats['estimated_costs'] = tp.get_estimated_task_costs()
        stats['real_costs'] = tp.get_real_task_costs()
        stats['map_name'] = map_name

        return stats


if __name__ == '__main__':
    def positive_integer(value):
        ivalue = int(value)
        if ivalue < 1:
            raise ArgumentTypeError("%s is an invalid positive int value" % value)
        return ivalue


    parser = ArgumentParser()

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
    args = parser.parse_args()

    print(args)

    map_file_names = glob(os.path.join(RootPath.get_root(), 'Environments', '*.yaml'))

    maps = []

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
                    agents_num = np.linspace(5, max_agents, args.agents, dtype=int).tolist()

                if args.starts_list:
                    start_num = args.starts_list
                elif args.starts == 1:
                    start_num = [max_starts]
                else:
                    start_num = np.linspace(5, max_starts, args.starts, dtype=int).tolist()

                if args.goals_list:
                    goal_num = args.goals_list
                elif args.goals == 1:
                    goal_num = [max_goals]
                else:
                    goal_num = np.linspace(5, max_goals, args.goals, dtype=int).tolist()

                map_yaml['agents_num'] = agents_num
                map_yaml['start_num'] = start_num
                map_yaml['goal_num'] = goal_num
                map_yaml['name'] = map_file_name.split('/')[-1].split('.')[0]

                maps.append(map_yaml)
            except yaml.YAMLError as exc:
                print(exc)

    maps_out = []

    gen_result = GenerateResults(maps, tasks_num, tasks_frequency, agents_num, start_num, goal_num)

    gen_result.generate_results()
