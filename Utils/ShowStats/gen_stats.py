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


def memorize_run_stats(old_stats, start_to_goal_times, start_to_pickup_times, pickup_to_goal_times,
                       actual_runtime: float, running_simulation: SimulationNewRecovery,
                       token_passing: TokenPassingRecovery):
    if running_simulation.time == 1:
        return {"costs": [0], "serv_times": [0], "start_to_pickup_times": [0], "pickup_to_goal_times": [0],
                "runtimes": [actual_runtime], "number_of_tasks": len(simulation.tasks)}
    else:
        pickup_to_goal_avgs = old_stats["pickup_to_goal_times"]
        start_to_pickup_avgs = old_stats["start_to_pickup_times"]
        serv_times = old_stats["serv_times"]
        runtimes = old_stats["runtimes"]
        costs = old_stats["costs"]

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

        return {"costs": costs, "serv_times": serv_times, "start_to_pickup_times": start_to_pickup_avgs,
                "pickup_to_goal_times": pickup_to_goal_avgs, "runtimes": runtimes,
                "number_of_tasks": len(simulation.tasks)}


def check_collisions(simulation: SimulationNewRecovery):
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
    parser.add_argument('-agents', help='Maximum number of agents. If the map can contain less than this number, the '
                                        'number of agents is the number of agents supported by the map',
                        type=int, default=100)
    parser.add_argument('-starts', help='Maximum number of starts. If the map can contain less than this number, the '
                                       'number of starts is the number of start supported by the map',
                        type=int, default=100)
    parser.add_argument('-goals', help='Maximum number of goals. If the map can contain less than this number, the '
                                       'number of goals is the number of goals supported by the map',
                        type=int, default=100)
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

    goal_locations = param['map']['goal_locations']
    start_locations = param['map']['start_locations']

    total_goals = len(goal_locations)
    total_starts = len(start_locations)

    goal_number = min(args.goals, len(param['map']['goal_locations']))
    start_number = min(args.starts, len(param['map']['start_locations']))

    # removing random n start from the list of start locations
    starts_to_delete = set(random.sample(range(total_starts), total_starts - start_number))
    start_locations = [start for startIndex, start in enumerate(start_locations) if startIndex not in starts_to_delete]

    # removing random n goals from the list of goal locations
    goals_to_delete = set(random.sample(range(total_goals), total_goals - goal_number))
    goal_locations = [goal for goalIndex, goal in enumerate(goal_locations) if goalIndex not in goals_to_delete]

    print("There are", len(start_locations), "start in the simulation.")
    print("There are", len(goal_locations), "goals in the simulation.")


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

    numpy.set_printoptions(threshold=sys.maxsize)

    dimensions = param['map']['dimensions']
    obstacles = param['map']['obstacles']
    non_task_endpoints = param['map']['non_task_endpoints']
    agents = param['agents']
    param['tasks'] = tasks

    total_agents = len(agents)
    agents_number = min(args.agents, total_agents)

    # removing random n agents from the list of agents
    agents_to_delete = set(random.sample(range(total_agents), total_agents - agents_number))
    agents = [agent for agentIndex, agent in enumerate(agents) if agentIndex not in agents_to_delete]

    print("There are", len(agents), "agents in the simulation.")

    print("Running Simulation with task distribution estimation...")

    simulation = SimulationNewRecovery(tasks, agents, task_distributions, True,
                                       args.update_td_every_t, last_task_time)
    tp = TokenPassingRecovery(agents, dimensions, max_time, obstacles, non_task_endpoints, simulation,
                              start_locations,
                              a_star_max_iter=args.a_star_max_iter, path_1_modified=True,
                              path_2_modified=True,
                              preemption_radius=args.preemption_distance,
                              preemption_duration=args.preemption_duration)

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

        stats = memorize_run_stats(stats, start_to_goal_times, start_to_pickup_times, pickup_to_goal_times, runtime,
                                   simulation, tp)

    check_collisions(simulation)

    print("Confronto tra costi teorici e costi reali: ")
    print("Costo stimato: ", sum(tp.get_estimated_task_costs().values()))
    print("Costo reale: ", sum(tp.get_real_task_costs().values()))

    print("Saving stats in stats_with_learning.csv...")

    dfStatsLearning = pd.DataFrame(stats, index=np.arange(0, simulation.get_time()))
    dfStatsLearning.index.name = "time"
    dfStatsLearning.to_csv("stats_with_learning.csv")

    print("Saving estimated and real costs per task in costs_learning.csv...")

    dfCostsLearning = pd.DataFrame({"estimated": tp.get_estimated_task_costs(), "real": tp.get_real_task_costs()})
    dfCostsLearning.index.name = "task_name"
    dfCostsLearning.to_csv("costs_learning.csv")

    print("Running Simulation with fixed task distribution...")

    simulation = SimulationNewRecovery(tasks, agents, task_distributions, False,
                                       args.update_td_every_t, last_task_time)
    tp = TokenPassingRecovery(agents, dimensions, max_time, obstacles, non_task_endpoints, simulation,
                              start_locations,
                              a_star_max_iter=args.a_star_max_iter, path_1_modified=True,
                              path_2_modified=True,
                              preemption_radius=args.preemption_distance,
                              preemption_duration=args.preemption_duration)

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

        stats = memorize_run_stats(stats, start_to_goal_times, start_to_pickup_times, pickup_to_goal_times, runtime,
                                   simulation, tp)

    check_collisions(simulation)

    print("Confronto tra costi teorici e costi reali: ")
    print("Costo stimato: ", sum(tp.get_estimated_task_costs().values()))
    print("Costo reale: ", sum(tp.get_real_task_costs().values()))

    print("Saving stats in stats_without_learning.csv...")

    dfStatsWithoutLearning = pd.DataFrame(stats, index=np.arange(0, simulation.get_time()))
    dfStatsWithoutLearning.index.name = "time"
    dfStatsWithoutLearning.to_csv("stats_without_learning.csv")

    print("Saving estimated and real costs per task in costs.csv...")

    dfCosts = pd.DataFrame({"estimated": tp.get_estimated_task_costs(), "real": tp.get_real_task_costs()})
    dfCosts.index.name = "task_name"
    dfCosts.to_csv("costs.csv")
