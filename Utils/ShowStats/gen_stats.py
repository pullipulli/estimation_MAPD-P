import argparse
import datetime
import json
import os
import random
import sys
from collections import defaultdict
from statistics import *

import numpy
import numpy as np
import pandas as pd
import yaml

import RootPath
from Simulation.TP_with_recovery import TokenPassingRecovery
from Simulation.simulation_new_recovery import SimulationNewRecovery


def memorize_run_stats(old_stats, actual_runtime: float, running_simulation: SimulationNewRecovery, token_passing: TokenPassingRecovery):
    if running_simulation.time == 1:
        return {"costs": [0], "serv_times": [0], "runtimes": [actual_runtime],
                "number_of_tasks": len(simulation.tasks)}
    else:
        serv_times = old_stats["serv_times"]
        runtimes = old_stats["runtimes"]
        costs = old_stats["costs"]

        cost = 0
        for path in running_simulation.actual_paths.values():
            cost = cost + len(path)

        serv_time = 0
        for task, end_time in token_passing.get_token()['completed_tasks_times'].items():
            serv_time = (end_time - token_passing.get_token()['start_tasks_times'][task])

        serv_times.append(serv_time)
        runtimes.append(actual_runtime)
        costs.append(cost)

        return {"costs": costs, "serv_times": serv_times, "runtimes": runtimes, "number_of_tasks": len(simulation.tasks)}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-preemption_distance', help='Maximum distance to be part of the preemption zone',
                        default=3, type=int)
    parser.add_argument('-preemption_duration', help='Preemption duration',
                        default=3, type=int)
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=5000, type=int)
    parser.add_argument('-tasks', help='Number of tasks',
                        default=80, type=int)
    parser.add_argument('-task_frequency',
                        help='Probability that a certain node will be assigned to a certain task '
                             'at a certain time-step',
                        default=0.2, type=float)
    parser.add_argument('-slow_factor', help='Slow factor of visualization', default=1, type=int)
    parser.add_argument('-gif', help='If it should a gif of the visualization', default=False, type=bool)
    parser.add_argument('-update_td_every_t', help='A integer >= 0 that changes how many time-steps should pass before '
                                                   'the simulation notify the task distribution changes to their '
                                                   'observers', default=15, type=int)
    args = parser.parse_args()

    number_of_tasks = args.tasks
    tasks_frequency = args.task_frequency

    with open(os.path.join(RootPath.get_root(), 'config.json'), 'r') as json_file:
        config = json.load(json_file)
    args.param = os.path.join(RootPath.get_root(), os.path.join(config['input_path'], config['input_name']))
    args.output = os.path.join(RootPath.get_root(), 'output.yaml')

    # Read from input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimensions = param['map']['dimensions']
    max_time = 10000
    dimensions = (dimensions[0], dimensions[1], max_time)
    task_distributions = [dict() for i in range(max_time)]
    tasks = []
    total = 0
    time = 0
    last_task_time = max_time
    while total < number_of_tasks:
        task_distribution = dict()
        tasks_now = numpy.random.poisson(tasks_frequency)
        locations = []
        if tasks_now > number_of_tasks - total:
            tasks_now = number_of_tasks - total
        while tasks_now > 0:
            tasks_now -= 1
            locations.append(random.choice(param['map']['start_locations']))
        for start in param['map']['start_locations']:
            if start in locations:
                probability = 1.0
                task_distribution[tuple(start)] = probability
                total += 1
                goal = random.choice(param['map']['goal_locations'])
                tasks.append(
                    {'start_time': time, 'start': start, 'goal': goal,
                     'task_name': 'task' + str(total)})
                last_task_time = time
        task_distributions[time] = task_distribution
        time += 1

    numpy.set_printoptions(threshold=sys.maxsize)

    dimensions = param['map']['dimensions']
    obstacles = param['map']['obstacles']
    non_task_endpoints = param['map']['non_task_endpoints']
    agents = param['agents']
    param['tasks'] = tasks

    print("Running Simulation with task distribution estimation...")

    simulation = SimulationNewRecovery(tasks, agents, task_distributions, True,
                                       args.update_td_every_t, last_task_time)
    tp = TokenPassingRecovery(agents, dimensions, max_time, obstacles, non_task_endpoints, simulation,
                              param['map']['start_locations'],
                              a_star_max_iter=args.a_star_max_iter, path_1_modified=True,
                              path_2_modified=True,
                              preemption_radius=args.preemption_distance,
                              preemption_duration=args.preemption_duration)

    runtime = 0
    stats = defaultdict(lambda: [])

    while tp.get_completed_tasks() != len(tasks):
        initialTime = datetime.datetime.now().timestamp()

        simulation.time_forward(tp)

        final = datetime.datetime.now().timestamp()
        runtime += final - initialTime

        stats = memorize_run_stats(stats, runtime, simulation, tp)

    print("Saving stats in stats_with_learning.csv...")

    dfStatsLearning = pd.DataFrame(stats, index=np.arange(0, simulation.get_time()))
    dfStatsLearning.index.name = "time"
    dfStatsLearning.to_csv("stats_with_learning.csv")

    print("Running Simulation with fixed task distribution...")

    simulation = SimulationNewRecovery(tasks, agents, task_distributions, False,
                                       args.update_td_every_t, last_task_time)
    tp = TokenPassingRecovery(agents, dimensions, max_time, obstacles, non_task_endpoints, simulation,
                              param['map']['start_locations'],
                              a_star_max_iter=args.a_star_max_iter, path_1_modified=True,
                              path_2_modified=True,
                              preemption_radius=args.preemption_distance,
                              preemption_duration=args.preemption_duration)

    runtime = 0
    stats = defaultdict(lambda: [])

    while tp.get_completed_tasks() != len(tasks):
        initialTime = datetime.datetime.now().timestamp()

        simulation.time_forward(tp)

        final = datetime.datetime.now().timestamp()
        runtime += final - initialTime

        stats = memorize_run_stats(stats, runtime, simulation, tp)

    print("Saving stats in stats_without_learning.csv...")

    dfStatsWithoutLearning = pd.DataFrame(stats, index=np.arange(0, simulation.get_time()))
    dfStatsWithoutLearning.index.name = "time"
    dfStatsWithoutLearning.to_csv("stats_without_learning.csv")

