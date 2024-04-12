import argparse
import datetime
import json
import os
import random
import sys
from statistics import *

import numpy
import yaml

import RootPath
from Simulation.TP_with_recovery import TokenPassingRecovery
from Simulation.simulation_new_recovery import SimulationNewRecovery

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m1', help='Use TP-m1 (Modify Path1)',
                        default=False, type=bool)
    parser.add_argument('-m2', help='Use TP-m2 (Modify Path2)',
                        default=False, type=bool)
    parser.add_argument('-preemption_distance', help='Maximum distance to be part of the preemption zone',
                        default=3, type=int)
    parser.add_argument('-preemption_duration', help='Preemption duration',
                        default=3, type=int)
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=5000, type=int)
    parser.add_argument('-tasks', help='Number of tasks',
                        default=80, type=int)
    parser.add_argument('-task_frequency',
                        help='Probability that a certain node will be assigned to a certain task at a certain instant of time',
                        default=0.2, type=float)
    args = parser.parse_args()

    number_of_tasks = args.tasks
    tasks_frequency = args.task_frequency

    print("Number of tasks:", number_of_tasks)
    print("Task frequency", tasks_frequency)

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
    dimensions = (dimensions[0], dimensions[1], 10000)  # third dimension is time (10 000 timesteps)
    task_distribution = numpy.zeros(dimensions)
    tasks = []
    total = 0
    time = 0
    while total < number_of_tasks:
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
                task_distribution[start[0], start[1], time] = probability
                total += 1
                goal = random.choice(param['map']['goal_locations'])
                tasks.append(
                    {'start_time': time, 'start': start, 'goal': goal,
                     'task_name': 'task' + str(total)})
            else:
                probability = 0.0
                task_distribution[start[0], start[1], time] = probability
        time += 1

    numpy.set_printoptions(threshold=sys.maxsize)

    dimensions = param['map']['dimensions']
    obstacles = param['map']['obstacles']
    non_task_endpoints = param['map']['non_task_endpoints']
    agents = param['agents']
    param['tasks'] = tasks

    # Simulate
    simulation = SimulationNewRecovery(tasks, agents, task_distribution)
    tp = TokenPassingRecovery(agents, dimensions, obstacles, non_task_endpoints, simulation,
                              param['map']['start_locations'],
                              a_star_max_iter=args.a_star_max_iter, path_1_modified=args.m1,
                              path_2_modified=args.m2,
                              preemption_radius=args.preemption_distance,
                              preemption_duration=args.preemption_duration)

    initialTime = datetime.datetime.now().timestamp()
    while tp.get_completed_tasks() != len(tasks):
        simulation.time_forward(tp)
        print("TASK DISTRIBUTION AT TIME ", simulation.get_time())
        print(simulation.get_task_distribution())
    final = datetime.datetime.now().timestamp()
    runtime = final - initialTime
    agents_cost = simulation.agents_cost
    for agent in simulation.actual_paths.keys():
        for t in range(len(simulation.actual_paths[agent]) - 1):
            for agent2 in simulation.actual_paths.keys():
                if agent2 != agent:
                    if simulation.actual_paths[agent][t] == simulation.actual_paths[agent2][t]:
                        print("Path collision agents" + str(agent) + " " + str(agent2) + str(
                            simulation.actual_paths[agent]) + str(
                            simulation.actual_paths[agent2]) + "at time " + str(
                            t))
                    if simulation.actual_paths[agent][t] == simulation.actual_paths[agent2][t + 1] and \
                            simulation.actual_paths[agent][t + 1] == simulation.actual_paths[agent2][t]:
                        print("Switch collision " + str(agent) + " " + str(agent2) + str(
                            simulation.actual_paths[agent]) + str(
                            simulation.actual_paths[agent2]) + "at time " + str(
                            t))

    costs = []
    replans = []
    service_times = []
    sim_times = []
    algo_times = []
    cost = 0
    for path in simulation.actual_paths.values():
        cost = cost + len(path)
    output = {'schedule': simulation.actual_paths, 'cost': cost,
              'completed_tasks_times': tp.get_completed_tasks_times(),
              }
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)

        cost = 0
        for path in simulation.actual_paths.values():
            cost = cost + len(path)
        costs.append(cost)
        serv_time = 0
        for task, end_time in tp.get_token()['completed_tasks_times'].items():
            serv_time = (end_time - tp.get_token()['start_tasks_times'][task])
            service_times.append(serv_time)
        print("Service time of this run: ", serv_time)
    avg_cost = mean(costs)
    avg_service_time = mean(service_times)
    print("Average service time:", avg_service_time, ". Standard deviation: ", ". Agents cost: ", agents_cost,
          ". Runtime cost:", runtime, ". Makespan:", simulation.time)

    # TODO insert moving agents visualization
