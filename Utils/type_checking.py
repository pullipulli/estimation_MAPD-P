from typing import Literal, TypeAlias, Any

Coordinate: TypeAlias = int
Time: TypeAlias = int
Name: TypeAlias = str
Location: TypeAlias = tuple[Coordinate, Coordinate]
Dimensions: TypeAlias = tuple[int, int, int]
TaskDistribution: TypeAlias = dict[Location, int]
Task: TypeAlias = dict[Literal["start", "goal", "start_time", "task_name"], Location | Time | Name]
Agent: TypeAlias = dict[Literal["name", "start"], str | Location]
Map: TypeAlias = dict[Literal["dimensions", "obstacles", "non_task_endpoints", "start_locations", "goal_locations"], Dimensions | list[Location]]
MapYaml: TypeAlias = dict[Literal["agents", "map"], list[Agent] | Map]
MapStats: TypeAlias = dict[Literal["map", "agents", "name", "size", "agents", "tasks", "agents_num", "start_num", "goal_num",], Name | Coordinate | list[Agent] | list[Task] | list[Agent] | Map]
RunId: TypeAlias = str
StatSimulation: TypeAlias = dict[Literal["costs", "serv_times", "start_to_pickup_times", "pickup_to_goal_times", "runtimes", "earth_mover_dist", "traffic", "agents", "pickup", "goal", "tasks", "task_frequency", "estimated_costs", "real_costs", "map_name", "last_task_time"], Any]
MapOutput: TypeAlias = dict[Literal["run_id", 'fixed', 'learning'], RunId | StatSimulation]
StatJson: TypeAlias = dict[Literal["maps", "tasks_num", "agents_num", "start_num", "goal_num", "tasks_frequency"], list[MapOutput] | list[int | float]]
