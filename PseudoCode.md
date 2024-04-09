# Token Passing Explanation

Questo file ha come obiettivo quello di spiegare con dello pseudocodice come funziona l'algoritmo Token Passing
contenuto tra questi file sorgenti. 

Nello specifico andrò a riassumere come funzionano:

- [demo.py](#demo)
- [Simulation/simulation_new_recovery.py](#simulation)
- [Simulation/TP_with_recovery.py](#tokenpassing)

## demo:

### config.json structure

```json
{
    "input_path": "<name of the folder where the input map is>",
    "input_name": "<name of the input map YAML file>"
}
```

### inputFile structure (the YAML input file)

```yaml
agents:
    - start: [0, 1]
      name: agent0
    - start: [0, 5]
      name: agent1
    # for each agent
map:
    dimensions: [25, 37]
    obstacles:
      - !!python/tuple [ 3, 2 ]
      - !!python/tuple [ 4, 2 ]
      # for each obstacle
    non_task_endpoints:
      - !!python/tuple [0, 1]
      - !!python/tuple [0, 5]
      # for each non task endpoint
    start_locations:
        -   [4, 1]
        -   [6, 1]
        # for each start location
    goal_locations:
        -   [0, 0]
        -   [0, 2]
        # for each goal location
```

### demo pseudocode

```python
    m1, m2, preemption_distance, preemption_duration,
        a_star_max_iter, tasks, task_frequency = ...args
        
    inputFile = it opens the input file from config.json
    
    agents = inputFile.agents
    obstacles = inputFile.map.obstacles
    non_task_endpoints = inputFile.map.non_task_endpoints
    start_locations = inputFile.map.start_locations
    goal_locations = param.map.goal_locations
    
    # In the real code this part is executed 20 times to average the results of the simulations
    
    # Creation of Task Distribution
    dimensions = inputFile.map.dimensions
    task_distribution = new [dimensions.x][dimensions.y][10000]
    tasks = []
    total = 0
    time = 0
    while total < tasks:
        tasks_now = poisson(task_frequency).random()
        locations = []
        if tasks_now > number_of_tasks - total:
            tasks_now = number_of_tasks - total
        while tasks_now > 0:
            tasks_now--
            newRandomStart = uniform(start_locations).random()
            locations.add(newRandomStart)
        for loc in start_locations:
            if loc in locations:
                task_distribution[loc.x, loc.y, time] = 1
                total += 1
                newTask = {'start_time': time, 'start': loc, 'goal': uniform(goal_locations).random(),
                     'task_name': 'task' + total}
                tasks.append(newTask)
            else:
                task_distribution[loc.x, loc.y, time] = 0
        time += 1
    
    simulation = SimulationNewRecovery(tasks, agents, task_distribution)
    
    tp = TokenPassingRecovery(agents, dimensions, obstacles, non_task_endpoints, simulation,
                              start_locations, a_star_max_iter, m1, m2,
                              preemption_distance, preemption_duration)
    ...                       
    # output of differents statistics
    ...
```

## Simulation

### SimulationNewRecovery Class

| Attribute         |                                                                                             Description                                                                                             |
|:------------------|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| tasks             |                                              List of tasks. A task is an object with: _start_time_, _start_ (the pickup location), _goal_, _task_name_                                              |
| task_distribution | Numpy 3D array. The the first two dimensions are the x and the y of the pickup location. The 3rd dimension is the time. The value is the probability of having a task on a certain pickup location. |
| agents            |                                                              List of agents. An agent has a _name_ and a _start_ location in the map.                                                               |
| time              |                                                                                     The time in the simulation                                                                                      |
| agents_cost       |                                                           The total cost of the simulation. It is incremented every time an agent moves.                                                            |
| start_times       |                                                                        (Never used) A list of the start times of the tasks.                                                                         |
| agents_pos_now    |                                                                      (Never used) A list of the actual position of the agents.                                                                      |
| agents_moved      |                                        (It should probably be a time_forward variable, not a class attribute) A set of the agents moved in the current time.                                        |
| actual_paths      |              A list of objects (with the agent name used as a key) with _t_ (time), _x_, _y_. This structure represents the entire path of an agent from the start of the simulation.               |
| algo_time         |                                                                                  The real time of the simulation.                                                                                   |

| Method                                  |                                                               Description                                                               |
|:----------------------------------------|:---------------------------------------------------------------------------------------------------------------------------------------:|
| get_time()                              |                                             It returns the current time of the simulation.                                              |
| get_algo_time()                         |                                 It returns the real time that has passed from the start of the program.                                 |
| get_actual_paths()                      |                                          Returns actual_paths attribute. _(See actual_paths)_                                           |
| get_new_tasks                           |                                        It returns a list of tasks available in the current time.                                        |
| [time_forward(algorithm)](#timeforward) | It increments the time of the simulation and calls the algorithm time_forward method. It also updates actual_paths and the agents_cost. |

#### SimulationNewRecovery Constructor

```
SimulationNewRecovery(tasks, agents, task_distribution)
```

#### time_forward

```python
def time_forward(algorithm):
    time++;
    start_time = current time
    algorithm.time_forward()
    algo_time = current time - start_time
    agents_to_move = set(agents)
    for agent in agents_to_move
        if (algorithm.get_token().agents[agent].path.length == 1) # the agent is currently idle in the same spot of the last timestep
            current_agent_pos = actual_paths[agent][-1]
            actual_paths[agent].append({'t': time, 'x': current_agent_pos.x, 'y': current_agent_pos.y})
        else if (algorithm.get_token().agents[agent].path.length > 1) # the agent has moved from his old position
            x_new = algorithm.get_token().agents[agent][1].x
            y_new = algorithm.get_token().agents[agent][1].y
            actual_paths[agent].append({'t': time, 'x': x_new, 'y': y_new})
            removes from the path the node where the agent was (and now it has moved). (It probably should be in a standalone method in the TokenPassingRecovery class)
            agents_cost++
```

## TokenPassing

### TokenPassingRecovery Class

| Attribute                 |                                                                                             Description                                                                                             |
|:--------------------------|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| agents                    |                                                              List of agents. An agent has a _name_ and a _start_ location in the map.                                                               |
| starts                    |                                                                                 List of possible pickup locations.                                                                                  |
| dimensions                |                                                                A tuple representing the dimensions of the map: (width, height, time)                                                                |
| path_1_modified           |                                                                                  Represents if TP should use TP-m1                                                                                  |
| path_2_modified           |                                                                                  Represents if TP should use TP-m2                                                                                  |
| preemption_radius         |                                                                         Maximum distance to be part of the preemption zone                                                                          |
| preemption_duration       |                                                  The number of timesteps in which the preemption is held after the agent reaches its destination.                                                   |
| preempted_locations       |                                                                           Preemption locations already claimed by agents.                                                                           |
| preemption_status         |                                                                       List of remaining preemption durations for each agent.                                                                        |
| preemption_zones          |                                                        For each pickup location, it has a list of the locations inside the preemption_radius                                                        |
| task_distribution         | Numpy 3D array. The the first two dimensions are the x and the y of the pickup location. The 3rd dimension is the time. The value is the probability of having a task on a certain pickup location. |
| obstacles                 |                                                                              List of the positions of fixed obstacles.                                                                              |
| non_task_endpoints        |                                       Locations that are neither pickup or delivery locations and can be used by agents as 'parking locations' by the agents.                                       |
| [token](#token-structure) |                                     The core of the TokenPassing algorithm. It contains all the useful information to execute the algorithm without conflicts.                                      |
| simulation                |                                                        A SimulationNewRecovery object that will execute the TokenPassing time_forward method                                                        |
| a_star_max_iter           |                                                                  Maximum number of states explored by the low-level algorithm (A*)                                                                  |

| Method                                                                                 |                                                                                                Description                                                                                                |
|:---------------------------------------------------------------------------------------|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| get_idle_agents()                                                                      |                                                                         It returns the agents that doesn't have a path assigned.                                                                          |
| admissible_heuristic(task_pos, agent_pos)                                              |                                                             It returns the manhattan distance between two positions (the task and the agent).                                                             |
| get_closest_task_name(available_tasks, agent_pos)                                      |                                                          Returns the name of the closest available task (if not physically occupied by an agent)                                                          |
| get_moving_obstacles_agents(agents, time_start)                                        |                                                                           Returns the list of moving agents before _time_start_                                                                           |
| get_idle_obstacles_agents(agents_paths, time_start)                                    |                                                                            Returns the list od idle agents before _time_start_                                                                            |
| check_safe_idle(agent_pos)                                                             |                                                         Returns True if the agent is not in current or available task locations (pickup or goal)                                                          |
| check_reachable_task_endpoint(task, current_agent)                                     |                                         Returns True if the current_agent can reach a task endpoint (using pickup, goal and preempted locations of other agents)                                          |
| get_closest_non_task_endpoint(agent_pos)                                               |                                                         Returns che closest non task endpoint to agent_pos that is not occupied by other agents.                                                          |
| get_preemption_zone(location)                                                          |                               Given a pickup location, it returns a list of the locations in the preemption zone (if the location is not a start, it returns an empty list)                               |
| get_preempted_locations()                                                              |                                                                       It returns the list of locations in all the preemption zones.                                                                       |
| [get_best_idle_location(agent_pos, best_task)](#getbestidlelocation)                   |                               It returns the best idle location in the preempted_locations. It returns the best_task if provided, and if it is in the preempted_locations.                                |
| update_ends(agent_pos)                                                                 |                                                            Remove _agent_pos_ from _token[path_ends]_ and _token[occupied_non_task_endpoints]_                                                            |
| get_agents_to_tasks_goals()                                                            |                                                                                    It returns the set of current goals                                                                                    |
| get_agents_to_tasks_starts_goals()                                                     |                                                                         It returns a set of the current goals and current starts                                                                          |
| get_completed_tasks()                                                                  |                                                                                      It returns the completed tasks                                                                                       |
| get_completed_tasks_times()                                                            |                                                                                It returns the times of the completed tasks                                                                                |
| get_token()                                                                            |                                                                                           It returns the token.                                                                                           |
| search(cbs)                                                                            |                                                 It returns tbe path obtained by the CBS algorithm. (_cbs_ must be initialized with the right Environment)                                                 |
| go_to_closest_non_task_endpoint(agent_name, agent_pos, all_idle_agents, path_modified) | Go to the closest non task endpoint. If path_modified is True, then the closest non task endpoint is obtained with get_best_idle_location (and it also updates the current occupied preemption locations) |
| get_random_close_cell(agent_pos, r)                                                    |                                 (Never used because it gets called only in deadlock_recovery) It returns the first random free cell near the _agent_pos_ in a radius _r_                                  |
| deadlock_recovery(agent_name, agent_pos, all_idle_agents, r)                           |                            (Never used) When called it increments the deadlock count for _agent_name_. If the counter is >= 5, then it calculates a new path to a random cell.                            |
| [time_forward()](#TPtimeforward)                                                       |                                        It updates the completed tasks, collect new tasks and assign the new tasks to the agents (and handle the preemption rights)                                        |



#### TokenPassingRecovery constructor

```
TokenPassingRecovery(agents, dimensions, obstacles, non_task_endpoints, simulation, starts, a_star_max_iter, path_1_modified, path_2_modified, preemption_radius, preemption_duration):
```

#### Token structure

- agents: for each agent it contains the current planned path
- tasks: for each task name, it contains a start location and a goal location
- start_tasks_times: for each task name, it contains the simulation time of its start time
- completed_tasks_times: for each task name, it contains the simulation time of its completion time
- agents_to_tasks: for each agent, it contains the task assigned to him (task_name, start, goal, predicted_cost)
- completed_tasks: the number of completed tasks
- path_ends: It contains the last locations (goals or idle locations) of the current paths of the agents
- occupied_non_task_endpoints: set of the occupied non task endpoints
- deadlock_count_per_agent: for each agent, it contains the deadlock count

#### get_best_idle_location

```python
def get_best_idle_location(agent_pos, best_task):
    actual_attractiveness = -1
    selected_best_task = [-1, -1]
    if best_task is not None and agent_pos in starts and best_task in preempted_locations[agent_pos]:
        return best_task
    for i in range(task_distribution.xNum):
        for j in range(task_distribution.yNum):
            task = [i, j]
            if task in starts and check_reachable_task_endpoint(task, agent_pos):
                possible_tasks_in_zone = 0
                distance_agent_to_task = admissible_heuristic(task, agent_pos) + 1
                preemption_zone = get_preemption_zone(task)
                for t in range(simulation.time + 1,
                               min(simulation.time + distance_agent_to_task + 1 + preemption_duration, task_distribution.timeNum)):
                    for location in preemption_zone:
                        possible_tasks_in_zone += task_distribution[location.x, location.y, t]
                attractiveness_of_task = possible_tasks_in_zone / (distance_agent_to_task + preemption_duration)
                if actual_attractiveness == -1:
                    actual_attractiveness = attractiveness_of_task
                    selected_best_task = task
                else:
                    if attractiveness_of_task > actual_attractiveness:
                        actual_attractiveness = attractiveness_of_task
                        selected_best_task = task
    if selected_best_task == [-1, -1]:
        return best_task
    
    if best_task is not None and 1 / (admissible_heuristic(agent_pos, best_task) + 1 + preemption_duration) >= actual_attractiveness:
        return best_task
    return selected_best_task
```

### TP.time_forward()

> Here I will assume that the function go_to_closest_non_task_endpoint has assigned:
> - A 'test' task_name when the agent is moving towards a preemption zone to look out for a possible new task
> - A 'safe_idle' task_name when the agent is moving towards a non task endpoint
> - Otherwise, the task is a normal task with a normal task_name

At high level, it can be divided in 3 sections:

```python
def time_forward():
    update_completed_tasks()
    collect_new_tasks()
    assign_new_tasks()
```

#### update_completed_tasks()

```python
for agent_name in token.agents:
    pos = simulation.actual_paths[agent_name][-1]
    if agent_name in token.agents_to_tasks and (pos.x, pos.y) == token.agents_to_tasks[agent_name].goal and len(token.agents[agent_name]) == 1 and token.agents_to_tasks[agent_name].task_name != 'safe_idle':
        if token.agents_to_tasks[agent_name].task_name != 'test':
            token.completed_tasks += 1
            token.completed_tasks_times[token.agents_to_tasks[agent_name].task_name] = simulation.get_time()
        token.agents_to_tasks.pop(agent_name)
    if agent_name in token.agents_to_tasks and (pos.x, pos.y) == token.agents_to_tasks[agent_name].goal and 
            len(token.agents[agent_name]) == 1 and token.agents_to_tasks[agent_name].task_name == 'safe_idle':
        token.agents_to_tasks.pop(agent_name)
```

#### collect_new_tasks()

```python
for t in simulation.get_new_tasks():
    token.tasks[t.task_name] = [t.start, t.goal]
    token.start_tasks_times[t.task_name] = simulation.get_time()
```

#### assign_new_tasks()

```python
idle_agents = get_idle_agents()
while len(idle_agents) > 0:
    agent_name = random.choice(idle_agents.keys())
    all_idle_agents = token.agents
    all_idle_agents.pop(agent_name)
    agent_pos = token.agents[agent_name][0]
    if agent_name in preempted_locations.keys() and agent_pos in preempted_locations[agent_name]:
        preemption_duration = preemption_status[agent_name]
        if preemption_duration == 0:
            preempted_locations[agent_name] = []
        else:
            preemption_status[agent_name] -= 1
    else:
        preemption_duration = 0
    available_tasks = {}
    for task_name, task in token.tasks.items():
        x1 = task.start not in token.path_ends.difference(agent_pos)
        x2 = task.goal not in token.path_ends.difference(agent_pos)
        x3 = task.start not in get_agents_to_tasks_goals()
        x4 = task.goal not in get_agents_to_tasks_goals()
        x5 = ((task.start not in get_preempted_locations() and preemption_duration == 0) or
              (agent_pos in starts and task.start in preempted_locations[agent_name]))
        if x1 and x2 and x3 and x4 and x5:
            available_tasks[task_name] = task

    if len(available_tasks) > 0 or agent_name in token.agents_to_tasks:
        already_calculated_path = False
        if agent_name in token.agents_to_tasks:
            closest_task_name = token.agents_to_tasks[agent_name].task_name

            closest_task = token.agents_to_tasks[agent_name]
            already_calculated_path = True
        else:
            closest_task_name = get_closest_task_name(available_tasks, agent_pos)
            closest_task = available_tasks[closest_task_name]
            if preemption_duration == 0 and path_1_modified and admissible_heuristic(get_best_idle_location(agent_pos, closest_task.start), agent_pos) < admissible_heuristic(closest_task.start, agent_pos):
                go_to_closest_non_task_endpoint(agent_name, agent_pos, all_idle_agents, True)
                already_calculated_path = True
        if not already_calculated_path:
            moving_obstacles_agents = get_moving_obstacles_agents(self.token.agents, 0)
            idle_obstacles_agents = get_idle_obstacles_agents(all_idle_agents.values(), 0)
            agent = {'name': agent_name, 'start': agent_pos, 'goal': closest_task.start}
            env = Environment(self.dimensions, [agent], obstacles | idle_obstacles_agents,
                              moving_obstacles_agents, a_star_max_iter=a_star_max_iter)
            cbs = CBS(env)
            path_to_task_start = search(cbs)
            if path_to_task_start:
                cost1 = env.compute_solution_cost(path_to_task_start)
                moving_obstacles_agents = get_moving_obstacles_agents(token.agents, cost1 - 1)
                idle_obstacles_agents = get_idle_obstacles_agents(all_idle_agents.values(), cost1 - 1)
                agent = {'name': agent_name, 'start': closest_task.start, 'goal': closest_task.goal}
                env = Environment(dimensions, [agent], obstacles | idle_obstacles_agents,
                                  moving_obstacles_agents, a_star_max_iter=a_star_max_iter)
                cbs = CBS(env)
                path_to_task_goal = search(cbs)
                if path_to_task_goal:
                    cost2 = env.compute_solution_cost(path_to_task_goal)
                    if agent_name not in token.agents_to_tasks:
                        token.tasks.pop(closest_task_name)
                        task = available_tasks.pop(closest_task_name)
                    else:
                        task = closest_task
                    last_step = path_to_task_goal[agent_name][-1]
                    update_ends(agent_pos)
                    preemption_status[agent_name] = 0
                    preempted_locations[agent_name] = [last_step.x, last_step.y]
                    token.path_ends.add([last_step.x, last_step.y])
                    token.agents_to_tasks[agent_name] = {'task_name': closest_task_name,
                                                                 'start': task.start,
                                                                 'goal': task.goal,
                                                                 'predicted_cost': cost1 + cost2}
                    token.agents[agent_name] = []
                    for el in path_to_task_start[agent_name]:
                        self.token.agents[agent_name].append([el.x, el.y])

                    token.agents[agent_name] = token.agents[agent_name][:-1]
                    for el in path_to_task_goal[agent_name]:
                        token.agents[agent_name].append([el.x, el.y])
    elif preemption_duration == 0:
        go_to_closest_non_task_endpoint(agent_name, agent_pos, all_idle_agents, path_2_modified)

```