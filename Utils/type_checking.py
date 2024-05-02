from typing import Dict, TypeAlias, List, Literal, Any

Coordinate: TypeAlias = int
Time: TypeAlias = int
Location: TypeAlias = List[Coordinate]
TaskDistribution = Dict[Location, float]
Task: TypeAlias = Dict[Literal["start", "goal", "start_time", "task_name"], Any]
Agent: TypeAlias = Dict[Literal["name", "start"], Any]
