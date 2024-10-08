# Use the gen_some_results arguments to set the variable parameters (This script works with the assumption that there is only one variable parameter at a time)
# Number arguments (generate n evenly spaced parameter numbers): -agents n, -tasks n, -tasks_frequency n, -starts n, -goals n
# Number arguments (generate n evenly spaced parameter numbers): -agents_list <args>, -tasks_list <args>, -tasks_frequency_list <args>, -starts_list <args>, -goals_list <args>
# This cell will take some time to run (obviously)
import os
import subprocess
from glob import glob
from multiprocessing import Process

import numpy as np
import yaml
from termcolor import colored

import RootPath
from Utils.ShowStats.gen_some_results import GenerateResults


def run_script(**kwargs):
    max_distance_traffic = 5
    if 'max_distance_traffic' in kwargs:
        max_distance_traffic = kwargs['max_distance_traffic']

    map_file_names = glob(os.path.join(RootPath.get_root(), 'Environments', '*.yaml'))
    maps = []

    tasks_list = [100]
    tasks_frequency_list = [1.0]
    td_update_list = [15]

    if 'tasks' in kwargs:
        tasks = kwargs['tasks']
        tasks_list = np.linspace(20, 100, tasks, dtype=int).tolist()
    elif 'tasks_list' in kwargs:
        tasks_list = kwargs['tasks_list']

    if 'tasks_frequency' in kwargs:
        tasks_frequency = kwargs['tasks_frequency']
        tasks_frequency_list = np.linspace(0.2, 2, tasks_frequency, dtype=float).tolist()
    elif 'tasks_frequency_list' in kwargs:
        tasks_frequency_list = kwargs['tasks_frequency_list']

    if 'td_update' in kwargs:
        td_update = kwargs['td_update']
        td_update_list = np.linspace(1, 30, td_update, dtype=int).tolist()
    elif 'td_update_list' in kwargs:
        td_update_list = kwargs['td_update_list']

    for map_file_name in map_file_names:
        with open(map_file_name, 'r') as map_file:
            try:
                map_yaml = yaml.load(map_file, Loader=yaml.FullLoader)

                max_agents = len(map_yaml['agents'])
                max_starts = len(map_yaml['map']['start_locations'])
                max_goals = len(map_yaml['map']['goal_locations'])

                agents_list = [max_agents]
                starts_list = [max_starts]
                goals_list = [max_goals]

                if 'agents' in kwargs:
                    agents = kwargs['agents']
                    min_agents = min(5, max_agents)
                    agents_list = np.linspace(min_agents, max_agents, agents, dtype=int).tolist()
                elif 'agents_list' in kwargs:
                    agents_list = kwargs['agents_list']

                if 'starts' in kwargs:
                    starts = kwargs['starts']
                    min_starts = min(5, max_starts)
                    starts_list = np.linspace(min_starts, max_starts, starts,  dtype=int).tolist()
                elif 'starts_list' in kwargs:
                    starts_list = kwargs['starts_list']

                if 'goals' in kwargs:
                    goals = kwargs['goals']
                    min_goals = min(5, max_goals)
                    goals_list = np.linspace(min_goals, max_goals, goals, dtype=int).tolist()
                elif 'goals_list' in kwargs:
                    goals_list = kwargs['goals_list']

                map_yaml['agents_num'] = agents_list
                map_yaml['start_num'] = starts_list
                map_yaml['goal_num'] = goals_list
                map_yaml['name'] = map_file_name.split('/')[-1].split('.')[0]

                maps.append(map_yaml)
            except yaml.YAMLError as exc:
                print(exc)

    generator = GenerateResults(maps, tasks_list, tasks_frequency_list, td_update_list, max_distance_traffic)
    generator.generate_results()


if __name__ == '__main__':
    processes = [Process(target=run_script, kwargs=dict(agents=3), name="agents"),
                 Process(target=run_script, kwargs=dict(tasks_list=[15, 30, 90]), name="tasks"),
                 Process(target=run_script, kwargs=dict(tasks_frequency_list=[0.2, 0.6, 2]), name="tasks_frequency"),
                 Process(target=run_script, kwargs=dict(starts=3), name="starts"),
                 Process(target=run_script, kwargs=dict(goals=3), name="goals"),
                 Process(target=run_script, kwargs=dict(td_update_list=[1, 10, 20]), name="td_update")]

    for process in processes:
        process.start()
        print(colored(f"Process {process.name} started", "green"))

    print("All Processes started")

    for process in processes:
        process.join()

    print("All Processes finished")

