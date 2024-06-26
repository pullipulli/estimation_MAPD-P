import math
import random
import time

from scipy.stats import wasserstein_distance

from Utils.observer_pattern import Observable
from Utils.type_checking import TaskDistribution, Task, Agent, Time


class SimulationNewRecovery(Observable):

    def __init__(self, tasks: list[Task], agents: list[Agent], task_distributions: list[TaskDistribution] = None,
                 learn_task_distribution=False,
                 update_time=30, last_task_time=10000):
        super().__init__()
        self.last_task_time = last_task_time
        self.update_time = update_time
        self.tasks = tasks
        self.task_distribution = task_distributions
        self.agents = agents
        self.learn_task_distribution = learn_task_distribution
        self.learned_task_distribution = dict()
        self.time = 0
        self.agents_cost = 0
        self.agents_moved = set()
        self.actual_paths = {}
        self.algo_time = 0
        self.initialize_simulation()

    def initialize_simulation(self):
        for agent in self.agents:
            self.actual_paths[agent['name']] = [{'t': 0, 'x': agent['start'][0], 'y': agent['start'][1]}]

    def time_forward(self, algorithm):
        self.time = self.time + 1
        start_time = time.time()
        algorithm.time_forward()
        self.algo_time += time.time() - start_time
        self.agents_moved = set()
        agents_to_move = self.agents
        random.shuffle(agents_to_move)
        for agent in agents_to_move:
            current_agent_pos = self.actual_paths[agent['name']][-1]

            if len(algorithm.get_token()['agents'][agent['name']]) == 1:
                # agent moved form his actual position
                self.agents_moved.add(agent['name'])
                self.actual_paths[agent['name']].append(
                    {'t': self.time, 'x': current_agent_pos['x'], 'y': current_agent_pos['y']})
            elif len(algorithm.get_token()['agents'][agent['name']]) > 1:
                task_name = algorithm.get_task_name_from_agent(agent['name'])

                if task_name is not None and task_name != 'test' and task_name != 'safe_idle':
                    algorithm.increment_real_task_cost(task_name)

                x_new = algorithm.get_token()['agents'][agent['name']][1][0]
                y_new = algorithm.get_token()['agents'][agent['name']][1][1]
                self.agents_moved.add(agent['name'])
                algorithm.get_token()['agents'][agent['name']] = algorithm.get_token()['agents'][
                                                                     agent['name']][1:]
                self.actual_paths[agent['name']].append({'t': self.time, 'x': x_new, 'y': y_new})
                self.agents_cost += 1
            else:
                self.actual_paths[agent['name']].append(
                    {'t': self.time, 'x': current_agent_pos['x'], 'y': current_agent_pos['y']})
                self.agents_cost += 1

        for task in self.get_new_tasks():
            start = task['start']
            if self.learned_task_distribution.get(tuple(start)) is None:
                self.learned_task_distribution[tuple(start)] = 1
            else:
                self.learned_task_distribution[tuple(start)] += 1

        if self.learn_task_distribution:
            if self.time > self.last_task_time:
                self.remove_observer(algorithm)
            elif self.update_time == 0:
                self.notify_observers()
            elif (self.time % self.update_time) == 0 and len(self.observers) != 0:
                self.notify_observers()

    def get_time(self) -> Time:
        return self.time

    def get_algo_time(self) -> float:
        return self.algo_time

    def get_actual_paths(self):
        return self.actual_paths

    def get_new_tasks(self) -> list[Task]:
        new = []
        for t in self.tasks:
            if t['start_time'] == self.time:
                new.append(t)
        return new

    def get_learned_task_distribution(self) -> TaskDistribution:
        freq_task_distribution = dict()

        for task in self.learned_task_distribution:
            freq_task_distribution[task] = self.learned_task_distribution[task] / len(
                self.learned_task_distribution)

        return freq_task_distribution

    def get_fixed_task_distribution_at_t(self, t) -> TaskDistribution:
        if len(self.task_distribution) <= t:
            return dict()

        return dict(self.task_distribution[t])

    def get_earth_mover_distance(self):
        learned_td = self.get_learned_task_distribution()
        fixed_td = self.get_fixed_task_distribution_at_t(self.time)

        if len(learned_td) == 0:
            return math.inf

        fixed_td = [fixed_td[k] if k in fixed_td else 0 for k in learned_td]
        learned_td = list(self.get_learned_task_distribution().values())

        return wasserstein_distance(fixed_td, learned_td)
