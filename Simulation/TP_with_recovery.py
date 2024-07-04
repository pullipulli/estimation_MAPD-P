"""
Token Passing with Recovery algorithm implementation.
This algorithm is used for task allocation in multi-agent systems. It is based on the Token Passing algorithm, but
it also includes a recovery mechanism in case of deadlocks.
"""
from __future__ import annotations

import math
import random
from collections import defaultdict
from math import fabs

from Simulation.CBS.cbs import CBS, Environment
from Utils.observer_pattern import Observer, Observable
from Utils.type_checking import Agent, Location, Dimensions, Token, LocationAtTime, PreemptionZones, PreemptedLocations
import typing

if typing.TYPE_CHECKING:
    from Simulation.simulation_new_recovery import SimulationNewRecovery


def admissible_heuristic(pos1: Location, pos2: Location):
    if pos1 is None or pos2 is None:
        return math.inf
    return fabs(pos1[0] - pos2[0]) + fabs(pos1[1] - pos2[1])

class TokenPassingRecovery(Observer):
    """
    The TokenPassingRecovery class is responsible for managing the token and the agents, and for executing the
    algorithm. It uses the CBS algorithm for path planning.
    """
    def __init__(self, agents: list[Agent], dimensions: Dimensions, max_time: int, obstacles: list[Location], non_task_endpoints: list[Location], simulation: SimulationNewRecovery,
                 starts: list[Location], a_star_max_iter=800000000, path_1_modified=False, path_2_modified=False,
                 preemption_radius=0, preemption_duration=0):
        self.agents = agents
        self.starts = starts
        self.dimensions = dimensions
        self.max_time = max_time
        self.path_1_modified = path_1_modified
        self.path_2_modified = path_2_modified
        self.preemption_radius = preemption_radius
        self.preemption_duration = preemption_duration
        self.learn_task_distribution = simulation.learn_task_distribution
        preemption_zones: PreemptionZones = {}
        self.preempted_locations: PreemptedLocations = {}
        self.preemption_status = {}
        for location in self.starts:
            zone = []
            for start in self.starts:
                if admissible_heuristic(start, location) <= preemption_radius:
                    zone.append(start)
            preemption_zones[tuple(location)] = zone
        self.preemption_zones = preemption_zones
        for agent_name in self.agents:
            self.preempted_locations[agent_name['name']] = []

        self.obstacles = set(obstacles)
        self.non_task_endpoints = non_task_endpoints
        if len(agents) > len(non_task_endpoints):
            # not well-formed instance
            exit(1)
        # TODO: Check all properties for well-formedness
        self.token: Token = {
            'agents': {},
            'tasks': {},
            'start_tasks_times': {},
            'pick_up_tasks_times': {},
            'completed_tasks_times': {},
            'estimated_cost_per_task': defaultdict(lambda: 0),
            'real_task_cost': defaultdict(lambda: 0),
            'agents_to_tasks': {},
            'completed_tasks': 0,
            'path_ends': set(),
            'occupied_non_task_endpoints': set(),
            'deadlock_count_per_agent': defaultdict(lambda: 0),
        }
        self.simulation = simulation
        self.a_star_max_iter = a_star_max_iter
        if self.learn_task_distribution:
            self.simulation.add_observer(self)
            self.learned_task_distribution = dict(simulation.get_learned_task_distribution())
        self.init_token()

    def init_token(self):
        """
        Initialize the token with the initial state of the simulation.
        :return:
        """
        for t in self.simulation.get_new_tasks():
            self.token['tasks'][t['task_name']] = [t['start'], t['goal']]
            self.token['start_tasks_times'][t['task_name']] = self.simulation.get_time()

        for a in self.agents:
            self.token['agents'][a['name']] = [a['start']]
            self.token['path_ends'].add(tuple(a['start']))

    def increment_real_task_cost(self, task_name):
        """
        Increment the real cost of a task.
        :param task_name:
        :return:
        """
        actual_cost = self.token['real_task_cost'].get(task_name, 0)

        self.token['real_task_cost'][task_name] = actual_cost + 1

    def get_task_name_from_agent(self, agent_name):
        """
        Get the name of the task assigned to an agent.
        :param agent_name:
        :return:
        """
        if agent_name in self.token['agents_to_tasks']:
            return self.token['agents_to_tasks'][agent_name]['task_name']
        return None

    def get_estimated_task_costs(self):
        """
        Get the estimated cost of each task.
        :return:
        """
        return dict(self.token['estimated_cost_per_task'])

    def get_real_task_costs(self):
        """
        Get the real cost of each task.
        :return:
        """
        return dict(self.token['real_task_cost'])

    def get_idle_agents(self):
        """
        Get the agents that are idle.
        :return:
        """
        agents = {}
        for name, path in self.token['agents'].items():
            if len(path) == 1:
                agents[name] = path
        return agents

    def get_closest_task_name(self, available_tasks, agent_pos):
        """
        Get the name of the closest task to an agent.
        :param available_tasks:
        :param agent_pos:
        :return:
        """
        closest = random.choice(list(available_tasks.keys()))
        occupied = []
        for path in self.token['agents'].values():
            if len(path) == 1:
                occupied.append(path[0])
        dist = admissible_heuristic(available_tasks[closest][0], agent_pos)
        for task_name, task in available_tasks.items():
            if admissible_heuristic(task[0], agent_pos) < dist and task[0] not in occupied:
                closest = task_name
        return closest

    def get_moving_obstacles_agents(self, agents, time_start):
        """
        Get the moving obstacles for the agents.
        :param agents:
        :param time_start:
        :return:
        """
        obstacles = {}
        for name, path in agents.items():
            if len(path) > time_start and len(path) > 1:
                for i in range(time_start, len(path)):
                    k = i - time_start
                    obstacles[(path[i][0], path[i][1], k)] = name
                    # Mark last element with negative time to later turn it into idle obstacle
                    if i == len(path) - 1:
                        obstacles[(path[i][0], path[i][1], -k)] = name
        return obstacles

    def get_idle_obstacles_agents(self, agents_paths, time_start):
        """
        Get the idle obstacles for the agents.
        :param agents_paths:
        :param time_start:
        :return:
        """
        obstacles = set()
        for path in agents_paths:
            if len(path) == 1:
                obstacles.add((path[0][0], path[0][1]))
            if 1 < len(path) <= time_start:
                obstacles.add((path[-1][0], path[-1][1]))
        return obstacles

    def check_safe_idle(self, agent_pos):
        """
        Check if an agent can safely idle in the agent_pos.
        :param agent_pos:
        :return:
        """
        for task_name, task in self.token['tasks'].items():
            if tuple(task[0]) == tuple(agent_pos) or tuple(task[1]) == tuple(agent_pos):
                return False
        for start_goal in self.get_agents_to_tasks_starts_goals():
            if tuple(start_goal) == tuple(agent_pos):
                return False
        return True

    def check_reachable_task_endpoint(self, task, current_agent):
        """
        Check if a task endpoint is reachable by an agent.
        :param task:
        :param current_agent:
        :return:
        """
        for task_name, task1 in self.token['tasks'].items():
            if tuple(task1[0]) == tuple(task) or tuple(task1[1]) == tuple(task):
                return False
        for start_goal in self.get_agents_to_tasks_starts_goals():
            if tuple(start_goal) == tuple(task):
                return False
        for agent_name, agent in self.token['agents'].items():
            if task in agent and agent[0] != current_agent:
                return False
            if agent[0] != current_agent and tuple(task) in self.preempted_locations[agent_name]:
                return False
        return True

    def get_closest_non_task_endpoint(self, agent_pos):
        """
        Get the closest non-task endpoint to an agent.
        :param agent_pos:
        :return:
        """
        dist = -1
        res = -1
        for endpoint in self.non_task_endpoints:
            exit = False
            for agent in self.token['agents'].values():
                if len(agent) == 1 and tuple(agent[0]) == endpoint:
                    exit = True
            if not exit and endpoint not in self.token['occupied_non_task_endpoints']:
                if dist == -1:
                    dist = admissible_heuristic(endpoint, agent_pos)
                    res = endpoint
                else:
                    tmp = admissible_heuristic(endpoint, agent_pos)
                    if tmp < dist:
                        dist = tmp
                        res = endpoint
        if res == -1:
            # Error in finding non-task endpoint, is instance well-formed?
            res = agent_pos
        return res

    def get_preemption_zone(self, location):
        """
        Get the preemption zone of a location.
        :param location:
        :return:
        """
        preempted_locations = self.get_preempted_locations()
        zone = []
        if location not in self.starts:
            return []
        for start in self.preemption_zones[tuple(location)]:
            if self.check_reachable_task_endpoint(start, location) and tuple(start) not in preempted_locations:
                zone.append(tuple(start))
        if location in self.starts and tuple(location) not in zone:
            zone.append(tuple(location))
        return list(set(zone))

    def get_preempted_locations(self):
        """
        Get the preempted locations.
        :return:
        """
        locations = []
        for zone in self.preempted_locations.values():
            for location in zone:
                locations.append(location)
        return list(locations)

    def get_best_idle_location(self, agent_pos, agent_name, best_task=None) -> [int, int]:
        """
        Get the best idle location for an agent.
        :param agent_pos:
        :param agent_name:
        :param best_task:
        :return:
        """
        best_idle_distance = -1
        best_idle = [-1, -1]
        if best_task is not None and tuple(agent_pos) in self.starts and best_task in self.preempted_locations[
             agent_name]:
            return best_task

        num_x = self.dimensions[0]
        num_y = self.dimensions[1]
        task_distr = dict()
        if self.learn_task_distribution:
            task_distr = self.learned_task_distribution

        for x in range(num_x):
            for y in range(num_y):
                start = [x, y]
                if start in self.starts and self.check_reachable_task_endpoint(start, agent_pos):
                    examined_loc_attractiveness = 0
                    distance = admissible_heuristic(start, agent_pos) + 1
                    preemption_zone = self.get_preemption_zone(start)

                    for t in range(self.simulation.get_time() + 1,
                                   min(self.simulation.get_time() + int(distance + 1) + self.preemption_duration,
                                       self.max_time)):
                        for location in preemption_zone:
                            if not self.learn_task_distribution:
                                task_distr = self.simulation.get_fixed_task_distribution_at_t(t)

                            examined_loc_attractiveness += task_distr.get(tuple(location), 0)
                    examined_distance = examined_loc_attractiveness / (distance + self.preemption_duration)


                    if best_idle_distance == -1:
                        best_idle_distance = examined_distance
                        best_idle = start
                    else:
                        if examined_distance > best_idle_distance:
                            best_idle_distance = examined_distance
                            best_idle = start

        if best_idle == [-1, -1]:
            return best_task

        if best_task is not None and 1 / (
                admissible_heuristic(agent_pos, best_task) + 1 + self.preemption_duration) >= best_idle_distance:
            return best_task

        return best_idle

    def update_ends(self, agent_pos):
        """
        Update the ends of the paths.
        :param agent_pos:
        :return:
        """
        if tuple(agent_pos) in self.token['path_ends']:
            self.token['path_ends'].remove(tuple(agent_pos))
        if tuple(agent_pos) in self.token['occupied_non_task_endpoints']:
            self.token['occupied_non_task_endpoints'].remove(tuple(agent_pos))

    def get_agents_to_tasks_goals(self):
        """
        Get the goals of the agents.
        :return:
        """
        goals = set()
        for el in self.token['agents_to_tasks'].values():
            goals.add(tuple(el['goal']))
        return goals

    def get_agents_to_tasks_starts_goals(self):
        """
        Get the starts and goals of the agents.
        :return:
        """
        starts_goals = set()
        for el in self.token['agents_to_tasks'].values():
            starts_goals.add(tuple(el['goal']))
            starts_goals.add(tuple(el['start']))
        return starts_goals

    def get_completed_tasks(self):
        """Get the number of completed tasks."""
        return self.token['completed_tasks']

    def get_completed_tasks_times(self):
        """Get the times of the completed tasks."""
        return self.token['completed_tasks_times']

    def get_pick_up_tasks_times(self):
        """Get the times of the pick-up tasks."""
        return self.token['pick_up_tasks_times']

    def get_start_tasks_times(self):
        """Get the times of the start tasks."""
        return self.token['start_tasks_times']

    def get_token(self):
        """Get the token."""
        return self.token

    def search(self, cbs):
        """Search for a path using the CBS algorithm."""
        path = cbs.search()
        return path

    def go_to_closest_non_task_endpoint(self, agent_name, agent_pos, all_idle_agents, path_modified):
        """
        Go to the closest non-task endpoint. (or the best idle location, depending on the path_modified parameter)
        :param agent_name:
        :param agent_pos:
        :param all_idle_agents:
        :param path_modified:
        :return:
        """
        best_idle_location = None
        closest_non_task_endpoint = None
        if path_modified:
            closest_non_task_endpoint = self.get_best_idle_location(agent_pos, agent_name)
            best_idle_location = closest_non_task_endpoint
        if closest_non_task_endpoint is None:  # or self.simulation.time > 100:
            closest_non_task_endpoint = self.get_closest_non_task_endpoint(agent_pos)
        moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], 0)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), 0)
        agent = {'name': agent_name, 'start': agent_pos, 'goal': closest_non_task_endpoint}
        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents, moving_obstacles_agents,
                          a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_non_task_endpoint: list[LocationAtTime] = self.search(cbs)

        if path_to_non_task_endpoint:
            self.update_ends(agent_pos)
            if best_idle_location is not None:
                last_step = path_to_non_task_endpoint[agent_name][-1]
                zone = self.get_preemption_zone([last_step['x'], last_step['y']])
                if agent_pos in self.preemption_zones[tuple([last_step['x'], last_step['y']])] and tuple(
                        agent_pos) not in zone:
                    zone.append(tuple(agent_pos))
                self.preempted_locations[agent_name] = zone
                self.preemption_status[agent_name] = self.preemption_duration
                self.token['path_ends'].add(tuple([last_step['x'], last_step['y']]))
                self.token['agents_to_tasks'][agent_name] = {'task_name': "test", 'start': agent_pos,
                                                             'goal': closest_non_task_endpoint, 'predicted_cost': 0}

            else:
                self.token['agents_to_tasks'][agent_name] = {'task_name': 'safe_idle', 'start': agent_pos,
                                                             'goal': closest_non_task_endpoint, 'predicted_cost': 0}
                self.token['occupied_non_task_endpoints'].add(tuple(closest_non_task_endpoint))
            self.token['agents'][agent_name] = []
            for el in path_to_non_task_endpoint[agent_name]:
                self.token['agents'][agent_name].append([el['x'], el['y']])
            if not self.token['agents'][agent_name]:
                self.token['agents'][agent_name].append(agent_pos)
        else:
            self.deadlock_recovery(agent_name, agent_pos, all_idle_agents, 4)

    def get_random_close_cell(self, agent_pos, r):
        """
        Get a random close cell to an agent.
        :param agent_pos:
        :param r:
        :return:
        """
        while True:
            cell = (
                agent_pos[0] + random.choice(range(-r - 1, r + 1)), agent_pos[1] + random.choice(range(-r - 1, r + 1)))
            if cell not in self.obstacles and cell not in self.token['path_ends'] and \
                    cell not in self.token['occupied_non_task_endpoints'] \
                    and cell not in self.get_agents_to_tasks_goals() \
                    and 0 <= cell[0] < self.dimensions[0] and 0 <= cell[1] < self.dimensions[1]:
                return cell

    def deadlock_recovery(self, agent_name, agent_pos, all_idle_agents, r):
        """
        Recover from a deadlock after a certain number of attempts.
        :param agent_name:
        :param agent_pos:
        :param all_idle_agents:
        :param r:
        :return:
        """
        print("DEADLOCK ", agent_name)
        self.token['deadlock_count_per_agent'][agent_name] += 1
        if self.token['deadlock_count_per_agent'][agent_name] >= 5:
            print("DEADLOCK RECOVERY ", agent_name)
            self.token['deadlock_count_per_agent'][agent_name] = 0
            random_close_cell = self.get_random_close_cell(agent_pos, r)
            moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], 0)
            idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), 0)
            agent = {'name': agent_name, 'start': agent_pos, 'goal': random_close_cell}
            env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents, moving_obstacles_agents,
                              a_star_max_iter=self.a_star_max_iter)
            cbs = CBS(env)
            path_to_non_task_endpoint = self.search(cbs)
            if path_to_non_task_endpoint:
                self.update_ends(agent_pos)
                self.token['agents'][agent_name] = []
                self.token['agents_to_tasks'][agent_name] = {'task_name': 'deadlock_recovery', 'start': agent_pos,
                                                             'goal': random_close_cell, 'predicted_cost': env.compute_solution_cost(path_to_non_task_endpoint)}

                for el in path_to_non_task_endpoint[agent_name]:
                    self.token['agents'][agent_name].append([el['x'], el['y']])
            else:
                print("Deadlock recovery failed ", agent_name)

    def time_forward(self):
        """
        Move the time forward.
        Update the completed tasks, collect new tasks, and assign them if possible.
        :return:
        """
        # Update completed tasks
        for agent_name in self.token['agents']:
            pos = self.simulation.actual_paths[agent_name][-1]
            if agent_name in self.token['agents_to_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.token['agents_to_tasks'][agent_name]['goal']) \
                    and len(self.token['agents'][agent_name]) == 1 and self.token['agents_to_tasks'][agent_name][
                'task_name'] != 'safe_idle':
                if (self.token['agents_to_tasks'][agent_name]['task_name'] != 'test' and
                        self.token['agents_to_tasks'][agent_name]['task_name'] != 'deadlock_recovery'):
                    self.token['completed_tasks'] = self.token['completed_tasks'] + 1
                    self.token['completed_tasks_times'][
                        self.token['agents_to_tasks'][agent_name]['task_name']] = self.simulation.get_time()
                self.token['agents_to_tasks'].pop(agent_name)
            if agent_name in self.token['agents_to_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.token['agents_to_tasks'][agent_name]['goal']) \
                    and len(self.token['agents'][agent_name]) == 1 and self.token['agents_to_tasks'][agent_name][
                'task_name'] == 'safe_idle':
                self.token['agents_to_tasks'].pop(agent_name)
        # Collect new tasks and assign them, if possible
        for t in self.simulation.get_new_tasks():
            self.token['tasks'][t['task_name']] = [t['start'], t['goal']]
            self.token['start_tasks_times'][t['task_name']] = self.simulation.get_time()
        idle_agents = self.get_idle_agents()
        while len(idle_agents) > 0:
            agent_name = random.choice(list(idle_agents.keys()))
            all_idle_agents = self.token['agents'].copy()
            all_idle_agents.pop(agent_name)
            agent_pos = idle_agents.pop(agent_name)[0]
            if agent_name in self.preempted_locations.keys() and tuple(agent_pos) in self.preempted_locations[
                 agent_name]:
                preemption_duration = self.preemption_status[agent_name]
                if preemption_duration == 0:
                    # Released preempted locations for agent_name
                    self.preempted_locations[agent_name] = []
                else:
                    self.preemption_status[agent_name] -= 1
            else:
                preemption_duration = 0
            available_tasks = {}
            for task_name, task in self.token['tasks'].items():

                x1 = tuple(task[0]) not in self.token['path_ends'].difference({tuple(agent_pos)})
                x2 = tuple(task[1]) not in self.token['path_ends'].difference({tuple(agent_pos)})
                x3 = tuple(task[0]) not in self.get_agents_to_tasks_goals()
                x4 = tuple(task[1]) not in self.get_agents_to_tasks_goals()
                x5 = ((tuple(task[0]) not in self.get_preempted_locations() and preemption_duration == 0) or
                      (agent_pos in self.starts and tuple(task[0]) in self.preempted_locations[agent_name]))
                if x1 and x2 and x3 and x4 and x5:
                    available_tasks[task_name] = task

            if len(available_tasks) > 0 or agent_name in self.token['agents_to_tasks']:
                x = None
                if agent_name in self.token['agents_to_tasks']:
                    closest_task_name = self.token['agents_to_tasks'][agent_name]['task_name']

                    closest_task = [self.token['agents_to_tasks'][agent_name]['start'],
                                    self.token['agents_to_tasks'][agent_name]['goal']]
                    x = 0
                else:
                    closest_task_name = self.get_closest_task_name(available_tasks, agent_pos)
                    closest_task = available_tasks[closest_task_name]
                    if preemption_duration == 0 and self.path_1_modified and admissible_heuristic(
                            self.get_best_idle_location(agent_pos, agent_name, closest_task[0]),
                            agent_pos) < admissible_heuristic(closest_task[0], agent_pos):
                        self.go_to_closest_non_task_endpoint(agent_name, agent_pos, all_idle_agents, True)
                        x = 0
                if x is None:
                    moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], 0)
                    idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), 0)
                    agent = {'name': agent_name, 'start': agent_pos, 'goal': closest_task[0]}
                    env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                                      moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
                    cbs = CBS(env)
                    path_to_task_start: list[LocationAtTime] = self.search(cbs)
                    if path_to_task_start:
                        cost1 = env.compute_solution_cost(path_to_task_start)

                        env = Environment(self.dimensions, [agent], self.obstacles, a_star_max_iter=self.a_star_max_iter)
                        cbs = CBS(env)
                        estimated_path_to_start = self.search(cbs)
                        estimated_cost1 = env.compute_solution_cost(estimated_path_to_start) - 1

                        # Use cost - 1 because idle cost is 1
                        moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], cost1 - 1)
                        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(),
                                                                               cost1 - 1)
                        agent = {'name': agent_name, 'start': closest_task[0], 'goal': closest_task[1]}
                        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                                          moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
                        cbs = CBS(env)
                        path_to_task_goal: list[LocationAtTime] = self.search(cbs)
                        if path_to_task_goal:
                            cost2 = env.compute_solution_cost(path_to_task_goal)
                            env = Environment(self.dimensions, [agent], self.obstacles,
                                              a_star_max_iter=self.a_star_max_iter)
                            cbs = CBS(env)
                            estimated_path_to_goal = self.search(cbs)
                            estimated_cost2 = env.compute_solution_cost(estimated_path_to_goal) - 1

                            if agent_name not in self.token['agents_to_tasks']:
                                self.token['tasks'].pop(closest_task_name)
                                task = available_tasks.pop(closest_task_name)
                            else:
                                task = closest_task
                            last_step = path_to_task_goal[agent_name][-1]
                            self.update_ends(agent_pos)
                            self.preemption_status[agent_name] = 0
                            self.preempted_locations[agent_name] = [tuple([last_step['x'], last_step['y']])]
                            self.token['path_ends'].add(tuple([last_step['x'], last_step['y']]))
                            self.token['agents_to_tasks'][agent_name] = {'task_name': closest_task_name,
                                                                         'start': task[0],
                                                                         'goal': task[1],
                                                                         'predicted_cost': cost1 + cost2}
                            self.token['estimated_cost_per_task'][closest_task_name] = estimated_cost1 + estimated_cost2
                            self.token['pick_up_tasks_times'][closest_task_name] = self.simulation.get_time()
                            self.token['agents'][agent_name] = []
                            for el in path_to_task_start[agent_name]:
                                self.token['agents'][agent_name].append([el['x'], el['y']])
                            # Don't repeat twice same step
                            self.token['agents'][agent_name] = self.token['agents'][agent_name][:-1]
                            for el in path_to_task_goal[agent_name]:
                                self.token['agents'][agent_name].append([el['x'], el['y']])
                        else:
                            self.deadlock_recovery(agent_name, agent_pos, all_idle_agents, 4)
                    else:
                        self.deadlock_recovery(agent_name, agent_pos, all_idle_agents, 4)
            elif preemption_duration == 0:
                self.go_to_closest_non_task_endpoint(agent_name, agent_pos, all_idle_agents, self.path_2_modified)

    def update(self, observable: Observable, *args, **kwargs):
        """
        Update the task_distribution if the simulation is learning.
        :param observable:
        :param args:
        :param kwargs:
        :return:
        """
        self.learned_task_distribution = dict(self.simulation.get_learned_task_distribution())

    def print(self, string: str):
        """Print a string with the current time."""
        print("TIME " + str(self.simulation.time) + ": " + string)
