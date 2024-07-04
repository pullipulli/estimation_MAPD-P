"""
AStar search
author: Ashwin Bose (@atb033)
author: Giacomo Lodigiani (@Lodz97)
"""
from __future__ import annotations

import heapq
from itertools import count
from typing import TYPE_CHECKING

from Utils.type_checking import AgentName

if TYPE_CHECKING:
    from cbs import Environment, State


class AStar:
    """
    Represents an A* search algorithm instance
    """
    def __init__(self, env: Environment):
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors
        self.max_iter = env.a_star_max_iter
        self.iter = 0

    @staticmethod
    def reconstruct_path(came_from: dict[State, State], current: State) -> list[State]:
        """
        Reconstruct the path from the came_from to the current position
        :param came_from:
        :param current:
        :return: The path from came_from to current
        """
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name: AgentName) -> list[State] | False:
        """
        Perform an A* search from the start location to the goal location for the given agent name and
        return the path (if found) or False otherwise
        """
        initial_state = self.agent_dict[agent_name]["start"]
        step_cost = 1

        closed_set = set()
        open_set = {initial_state}

        came_from = {}

        g_score = {initial_state: 0}

        f_score = {}
        h_score = self.admissible_heuristic(initial_state, agent_name)
        f_score[initial_state] = h_score

        heap = []
        index = count(0)
        heapq.heappush(heap, (f_score[initial_state], h_score, next(index), initial_state))

        while open_set and (self.max_iter == -1 or self.iter < self.max_iter):
            self.iter = self.iter + 1
            if self.iter == self.max_iter:
                print('Low level A* - Maximum iteration reached')
            current = heapq.heappop(heap)[3]

            if self.is_at_goal(current, agent_name):
                return self.reconstruct_path(came_from, current)

            open_set -= {current}
            closed_set |= {current}

            neighbor_list = self.get_neighbors(current)

            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                h_score = self.admissible_heuristic(neighbor, agent_name)
                f_score[neighbor] = g_score[neighbor] + h_score
                heapq.heappush(heap, (f_score[neighbor], h_score, next(index), neighbor))
        return False
