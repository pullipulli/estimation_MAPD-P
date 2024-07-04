"""
Python implementation of Conflict-based search
author: Ashwin Bose (@atb033)
author: Giacomo Lodigiani (@Lodz97)
"""
from __future__ import annotations

import sys
import argparse
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy

from Simulation.CBS.a_star import AStar
from Utils.type_checking import Dimensions, Agent, AgentName, LocationAtTime

sys.path.insert(0, '../')


class Location(object):
    """
    Class to represent a location in the grid
    """
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return str((self.x, self.y))


class State(object):
    """
    Class to represent a state (location and time) of an agent
    """
    def __init__(self, time: int | float, location: Location):
        self.time = time
        self.location = location

    def __eq__(self, other: State):
        return self.time == other.time and self.location == other.location

    def __hash__(self):
        return hash(str(self.time) + str(self.location.x) + str(self.location.y))

    def is_equal_except_time(self, state: State) -> bool:
        """
        Check if two states are equal except time
        :param state: the other state to compare
        :return: True if states are equal except time, False otherwise
        """
        return self.location == state.location

    def __str__(self):
        return str((self.time, self.location.x, self.location.y))


class Conflict(object):
    """
    Class to represent a conflict between agents
    """
    VERTEX = 1
    EDGE = 2

    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
            ', ' + str(self.location_1) + ', ' + str(self.location_2) + ')'


class VertexConstraint(object):
    """
    Class to represent a vertex constraint at a certain time
    """
    def __init__(self, time: int | float, location: Location):
        self.time = time
        self.location = location

    def __eq__(self, other: VertexConstraint):
        return self.time == other.time and self.location == other.location

    def __hash__(self):
        return hash(str(self.time) + str(self.location))

    def __str__(self):
        return '(' + str(self.time) + ', ' + str(self.location) + ')'


class EdgeConstraint(object):
    """
    Class to represent an edge constraint between two locations at a certain time
    """
    def __init__(self, time: int | float, location_1: Location, location_2: Location):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2

    def __eq__(self, other: EdgeConstraint):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2

    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))

    def __str__(self):
        return '(' + str(self.time) + ', ' + str(self.location_1) + ', ' + str(self.location_2) + ')'


class Constraints(object):
    """
    Class to represent a set of vertex and edge constraints
    """
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other: Constraints) -> None:
        """
        Add constraints from another object
        :param other: Another Constraints Object
        """
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints]) + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])


class Environment(object):
    """
    Class to represent the environment in which the agents are moving
    """
    def __init__(self, dimension: Dimensions, agents: list[Agent], obstacles: set[Location],
                 moving_obstacles: dict[LocationAtTime, Agent] = None, a_star_max_iter=-1):
        if moving_obstacles is None:
            moving_obstacles = []
        self.dimension = dimension
        self.obstacles = obstacles
        self.moving_obstacles = moving_obstacles
        self.a_star_max_iter = a_star_max_iter

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict: dict[Agent, Constraints] = {}

        self.a_star = AStar(self)

    def get_neighbors(self, state: State) -> list[State]:
        """
        Get the neighbors of a state
        :param state:
        :return: a list of states that are neighbors of the input state
        """
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y + 1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y - 1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x - 1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x + 1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors

    def get_first_conflict(self, solution: dict[Agent, list[State]]):
        """
        Get the first conflict in the solution (if any)
        :param solution:
        :return:
        """
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t + 1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t + 1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    @staticmethod
    def create_constraints_from_conflict(conflict: Conflict):
        """
        Create constraints from a conflict
        :param conflict:
        :return:
        """
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    @staticmethod
    def get_state(agent_name: AgentName, solution, t):
        """
        Get the state of an agent at a certain time (or the last state)
        :param agent_name:
        :param solution:
        :param t:
        :return:
        """
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def get_all_obstacles(self, time: int | float):
        """
        Get all obstacles at a certain time
        :param time:
        :return:
        """
        all_obs = set()
        for o in self.moving_obstacles:
            if o[2] < 0 and time >= -o[2]:
                all_obs.add((o[0], o[1]))
        return self.obstacles | all_obs

    def state_valid(self, state: State) -> bool:
        """
        Check if a state is valid (inside the grid, not an obstacle, not a vertex constraint)
        :param state:
        :return: True if the state is valid, False otherwise
        """
        return 0 <= state.location.x < self.dimension[0] \
            and 0 <= state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.get_all_obstacles(state.time) \
            and (state.location.x, state.location.y, state.time) not in self.moving_obstacles

    def transition_valid(self, state_1, state_2):
        """
        Check if a transition between two states is valid (if there is no edge constraint between them)
        :param state_1:
        :param state_2:
        :return:
        """
        tup_1 = (state_1.location.x, state_1.location.y, state_2.time)
        tup_2 = (state_2.location.x, state_2.location.y, state_1.time)
        if tup_1 in self.moving_obstacles and tup_2 in self.moving_obstacles and \
                self.moving_obstacles[tup_1] == self.moving_obstacles[tup_2]:
            return False
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        """
        Admissible heuristic for the A* search (Manhattan distance to the goal)
        :param state:
        :param agent_name:
        :return:
        """
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)

    def is_at_goal(self, state, agent_name):
        """
        Check if a state is at the goal of an agent
        :param state:
        :param agent_name:
        :return:
        """
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        """
        Create a dictionary of agents (self.agent_dict) where for each agent name we have the start and goal State
        """
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']: {'start': start_state, 'goal': goal_state}})

    def compute_solution(self):
        """
        Compute the solution for all agents
        :return:
        """
        solution: dict[Agent, list[Location]] = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent: local_solution})
        return solution

    @staticmethod
    def compute_solution_cost(solution: dict[Agent, list[Location]]):
        """
        Compute the cost of a solution
        :param solution:
        :return:
        """
        return sum([len(path) for path in solution.values()])


class HighLevelNode(object):
    """
    Class to represent a node in the high-level search
    """
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other: HighLevelNode):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash(self.cost)

    def __lt__(self, other):
        return self.cost < other.cost


class CBS(object):
    """
    Class to represent the Conflict-based search algorithm
    """
    def __init__(self, environment: Environment):
        self.env = environment
        self.open_set = set()
        self.closed_set = set()

    def search(self):
        """
        Perform the CBS search algorithm and return the solution (if found)
        :return:
        """
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}
        start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}

        while self.open_set:
            P = min(self.open_set)
            self.open_set -= {P}
            self.closed_set |= {P}

            self.env.constraint_dict = P.constraint_dict
            conflict_dict = self.env.get_first_conflict(P.solution)
            if not conflict_dict:
                # print("Low level CBS - Solution found")

                return self.generate_plan(P.solution)

            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                self.env.constraint_dict = new_node.constraint_dict
                new_node.solution = self.env.compute_solution()
                if not new_node.solution:
                    continue
                new_node.cost = self.env.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}

    @staticmethod
    def generate_plan(solution):
        """
        Generate a plan from the solution (a dictionary of agent names and their paths)
        :param solution:
        :return:
        """
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t': state.time, 'x': state.location.x, 'y': state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-param", help="input file containing map and obstacles")
    parser.add_argument("-output", help="output file with the schedule")
    args = parser.parse_args()

    if args.param is None:
        args.param = 'input.yaml'
        args.output = 'output.yaml'

    # Read from input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]
    obstacles = param["map"]["obstacles"]
    agents = param['agents']

    env = Environment(dimension, agents, obstacles, a_star_max_iter=1000)

    # Searching
    cbs = CBS(env)
    solution = cbs.search()
    if not solution:
        # print("Solution not found")
        exit(0)

    # Write to output file
    with open(args.output, 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    output["schedule"] = solution
    output["cost"] = env.compute_solution_cost(solution)
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)
