"""
This module contains type aliases and typed dictionaries used for type checking in the project.
"""
from __future__ import annotations
from typing import TypeAlias, TypedDict, Literal, NotRequired

Coordinate: TypeAlias = int
Time: TypeAlias = int
TaskName: TypeAlias = str
AgentName: TypeAlias = str
RunId: TypeAlias = str
MapName: TypeAlias = str

Location: TypeAlias = tuple[Coordinate, Coordinate] | list[Coordinate]

Dimensions: TypeAlias = tuple[int, int, int] | tuple[int, int]

LocationAtTime = TypedDict('LocationAtTime', {'x': Coordinate, 'y': Coordinate, Literal['time', 't']: Time})

TaskDistribution: TypeAlias = dict[Location | tuple[Location], int | float]

Task = TypedDict('Task', {'start': Location, 'goal': Location, 'start_time': Time, 'task_name': TaskName})

Agent = TypedDict('Agent', {'name': AgentName, 'start': Location, 'goal': NotRequired[Location]})

Map = TypedDict('Map', {'dimensions': Dimensions, 'obstacles': list[Location], 'non_task_endpoints': list[Location],
                        'start_locations': list[Location], 'goal_locations': list[Location]})

MapYaml = TypedDict('MapYaml', {'map': Map, 'agents': list[Agent]})

MapStats = TypedDict('MapStats', {'map': Map, 'name': MapName, 'agents': list[Agent], 'tasks': list[Task],
                                  'agents_num': int, 'start_num': int, 'goal_num': int})

StatSimulation = TypedDict('StatSimulation', {'costs': list[float], 'serv_times': list[float],
                                              'start_to_pickup_times': list[float], 'pickup_to_goal_times': list[float],
                                              'runtimes': list[float], 'earth_mover_dist': list[float],
                                              'traffic': NotRequired[list[list[int]]], 'agents': NotRequired[int],
                                              'pickup': NotRequired[int], 'goal': NotRequired[int],
                                              'tasks': NotRequired[int], 'task_frequency': NotRequired[float],
                                              'estimated_costs': NotRequired[list[int]],
                                              'real_costs': NotRequired[list[int]],
                                              'map_name': NotRequired[MapName], 'last_task_time': NotRequired[Time]})

MapOutput = TypedDict('MapOutput', {'run_id': RunId, 'fixed': StatSimulation, 'learning': StatSimulation})

StatJson = TypedDict('StatJson', {'maps': list[MapOutput], 'tasks_num': list[int], 'agents_num': list[int],
                                  'start_num': list[int], 'goal_num': list[int], 'tasks_frequency': list[float]})

Token = TypedDict('Token', {'agents': dict[AgentName, list[Location]],
                            'tasks': dict[TaskName, list[Location] | tuple[Location]],
                            'start_tasks_times': dict[TaskName, Time], 'pick_up_tasks_times': dict[TaskName, Time],
                            'completed_tasks_times': dict[TaskName, Time],
                            'estimated_cost_per_task': dict[TaskName, int],
                            'real_task_cost': dict[TaskName, int], 'agents_to_tasks': dict[AgentName, Task],
                            'completed_tasks': int, 'path_ends': set[Location, ...],
                            'occupied_non_task_endpoints': set[tuple[Location]],
                            'deadlock_count_per_agent': dict[AgentName, int]})

PreemptionZones = TypedDict('PreemptionZones', {Location: list[Location]})

PreemptedLocations = TypedDict('PreemptedLocations', {AgentName: list[Location]})
