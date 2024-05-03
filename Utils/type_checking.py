from typing import Literal

type Coordinate = int
type Time = int
type Name = str
type Location = list[Coordinate]
type TaskDistribution = dict[Location, int]
type Task = dict[Literal["start", "goal", "start_time", "task_name"], Location | Time | Name]
type Agent = dict[Literal["name", "start"], str | Location]
