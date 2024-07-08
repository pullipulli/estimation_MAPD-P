"""
SimulationNewRecovery class
This class is used to simulate the execution of a MAPD solving algorithm.
"""

import math
import random
import time

from scipy.stats import wasserstein_distance

from Simulation.TP_with_recovery import admissible_heuristic
from Utils.observer_pattern import Observable
from Utils.type_checking import TaskDistribution, Task, Agent, Time, AgentName, LocationAtTime


class SimulationNewRecovery(Observable):
    """
    This class is used to simulate the execution of the algorithm with the recovery mechanism.
    The simulation is done by moving the agents (it also updates the traffic matrix).
    The simulation can also learn the task distribution and update it every update_time.

    :param tasks: list of tasks
    :param agents: list of agents
    :param task_distributions: list of task distributions at each timestep
    :param learn_task_distribution: boolean to know if the task distribution should be learned or not
    :param update_time: time between two updates of the learned task distribution (if learn_task_distribution is True)
    :param last_task_time: time when there are no more tasks
    :param max_time: maximum limit of the simulation time
    :param max_distance_traffic: maximum distance for the traffic matrix

    Attributes:
    - :class:`tasks`: list of tasks
    - :class:`agents`: list of agents
    - :class:`task_distribution`: list of task distributions at each timestep
    - :class:`learn_task_distribution`: boolean to know if the task distribution should be learned or not
    - :class:`learned_task_distribution`: learned task distribution
    - :class:`time`: current simulation time
    - :class:`agents_cost`: sum of agents cost (number of agents that moved or stayed at the same position at each timestep)
    - :class:`actual_paths`: actual paths of the agents at each timestep (x, y coordinates)
    - :class:`algo_time`: real time spent in the algorithm
    - :class:`max_time`: maximum limit of the simulation time
    - :class:`max_distance_traffic`: maximum distance for the traffic matrix
    - :class:`traffic_matrix`: traffic matrix at each timestep (number of couples of agents at each distance)
    - :class:`update_time`: time between two updates of the learned task distribution (if learn_task_distribution is True)
    - :class:`last_task_time`: time when there are no more tasks
    """

    def __init__(self, tasks: list[Task], agents: list[Agent], task_distributions: list[TaskDistribution] = None,
                 learn_task_distribution=False,
                 update_time=30, last_task_time=10000, max_time=10000, max_distance_traffic=5):
        super().__init__()
        self.last_task_time = last_task_time
        self.update_time = update_time
        self.tasks = tasks
        self.task_distribution = task_distributions
        self.agents = agents
        self.learn_task_distribution = learn_task_distribution
        self.learned_task_distribution: TaskDistribution = dict()
        self.time = 0
        self.agents_cost = 0
        self.actual_paths: dict[AgentName, list[LocationAtTime]] = {}
        self.algo_time: float = 0
        self.max_time = max_time
        self.max_distance_traffic = max_distance_traffic
        self.traffic_matrix: list[list[int]] = []
        self.initialize_simulation()

    def initialize_simulation(self) -> None:
        """Initialize the simulation by setting the initial paths of the agents in actual_paths."""
        for agent in self.agents:
            self.actual_paths[agent['name']] = [{'t': 0, 'x': agent['start'][0], 'y': agent['start'][1]}]

    def time_forward(self, algorithm) -> None:
        """
        Move the agents and update the traffic matrix.
        Update the learned task distribution if needed.
        :param algorithm: the algorithm to use to move the agents
        """
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

        newRow = [0] * self.max_distance_traffic

        for agent1 in self.agents:
            for agent2 in self.agents:
                if agent1['name'] != agent2['name']:
                    agent1_pos = self.actual_paths[agent1['name']][-1]
                    agent2_pos = self.actual_paths[agent2['name']][-1]
                    distance = math.floor(admissible_heuristic((agent1_pos['x'], agent1_pos['y']),
                                                               (agent2_pos['x'], agent2_pos['y'])))

                    if 0 < distance <= self.max_distance_traffic:
                        newRow[distance-1] += 0.5  # couples of agents (so we need to add 2 0.5 to have 1 couple)

        self.traffic_matrix.append(newRow)

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
        """Get the current simulation time."""
        return self.time

    def get_algo_time(self) -> float:
        """Get the real time spent in the algorithm."""
        return self.algo_time

    def get_actual_paths(self):
        """Get the actual paths of the agents."""
        return self.actual_paths

    def get_new_tasks(self) -> list[Task]:
        """
        Get the tasks that start at the current time.
        :return: the tasks that start at the current time
        """
        new = []
        for t in self.tasks:
            if t['start_time'] == self.time:
                new.append(t)
        return new

    def get_learned_task_distribution(self) -> TaskDistribution:
        """
        Get the learned (relative) task distribution.
        :return: the learned task distribution
        """
        freq_task_distribution = dict()

        for task in self.learned_task_distribution:
            freq_task_distribution[task] = self.learned_task_distribution[task] / self.get_number_of_assigned_tasks()

        return freq_task_distribution

    def get_fixed_task_distribution_at_t(self, t) -> TaskDistribution:
        """
        Get the fixed task distribution at time t.
        :param t: time
        :return: the fixed task distribution at t
        """
        if len(self.task_distribution) <= t:
            return dict()

        return dict(self.task_distribution[t])

    def get_number_of_assigned_tasks(self) -> int:
        """
        Get the number of assigned tasks.
        :return: the number of assigned tasks
        """
        return sum(self.learned_task_distribution.values())

    def get_earth_mover_distance(self) -> float:
        """
        Compute the earth mover distance between the fixed task distribution and the learned task distribution.
        :return: the earth mover distance
        """
        learned_td = self.get_learned_task_distribution()
        fixed_td = self.get_fixed_task_distribution_at_t(self.time)
        fixed_tasks_at_t = sum(fixed_td.values())

        if len(learned_td) == 0:
            return math.inf

        fixed_td = [fixed_td[k] / fixed_tasks_at_t if k in fixed_td else 0 for k in learned_td]
        learned_td = list(self.get_learned_task_distribution().values())

        return wasserstein_distance(fixed_td, learned_td)
