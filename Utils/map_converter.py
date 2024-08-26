"""
This script is used to convert ASCII maps to YAML format.
It does NOT generate a well formed mapd instance, but it is a good basis to start from.
"""
import argparse
import json
import os
import re
import yaml
import RootPath
import random
from math import fabs

def near_obstacle(location, obstacles):
    num_obstacles = 0
    for obstacle in obstacles:
        if fabs(location[0] - obstacle[0]) + fabs(location[1] - obstacle[1]) == 1:
            num_obstacles += 1

    return num_obstacles

def is_valid_agent_location(other_agents, agent_location, occupied_locations):
    if agent_location in other_agents:
        return False

    for location in other_agents:
        if abs(location[0] - agent_location[0]) <= 2 and abs(location[1] - agent_location[1]) <= 2:
            return False

    for location in occupied_locations:
        if abs(location[0] - agent_location[0]) <= 3 and abs(location[1] - agent_location[1]) <= 3:
            return False

    return True


def random_agents_locations(free_locations, num_agents, occupied_locations):
    free_locs = list(free_locations)
    agent_locations = []

    for i in range(num_agents):
        agent_location = random.choice(free_locs)
        free_locs.remove(agent_location)

        if is_valid_agent_location(agent_locations, agent_location, occupied_locations):
            agent_locations.append(agent_location)
        else:
            i -= 1

    return agent_locations


def random_start_or_goal_locations(obstacles, free_locations, num_locations=50):
    three_obstacles = set()
    one_obstacle = set()
    zero_obstacles = set()
    assigned_locations = set()

    for location in free_locations:
        if near_obstacle(location, obstacles) == 3:
            three_obstacles.add(tuple(location))
        elif near_obstacle(location, obstacles) == 1:
            one_obstacle.add(tuple(location))
        else:
            zero_obstacles.add(tuple(location))

    remaining_locations = num_locations - len(three_obstacles)

    if remaining_locations <= 0:
        return list(three_obstacles)
    else:
        for location in three_obstacles:
            assigned_locations.add(location)

    remaining_locations -= len(one_obstacle)

    if remaining_locations <= 0:
        return list(assigned_locations) + list(one_obstacle)
    else:
        for location in one_obstacle:
            assigned_locations.add(tuple(location))

    last_locations = []

    if len(assigned_locations) < num_locations:
        last_locations = random.sample(sorted(zero_obstacles), remaining_locations)

    for location in last_locations:
        assigned_locations.add(tuple(location))

    return list(assigned_locations)


def map_converter(map_name: str):
    yaml_dic = {}
    with open(os.path.join(os.path.join(RootPath.get_root(), 'Benchmarks'), map_name + '.map')) as ascii_map:
        ascii_map.readline()
        h = int(re.findall(r'\d+', ascii_map.readline())[0])
        w = int(re.findall(r'\d+', ascii_map.readline())[0])
        yaml_dic['agents'] = []
        yaml_dic['map'] = {'dimensions': [w, h], 'obstacles': [], 'non_task_endpoints': [],
                           'start_locations': [], 'goal_locations': []}

        ascii_map.readline()
        free_locations = []
        obstacles = []
        for i in range(h - 1, -1, -1):
            line = ascii_map.readline()
            print(line)
            for j in range(w):
                if line[j] == '@' or line[j] == 'T':
                    obstacles.append((j, i))
                else:
                    free_locations.append([j, i])

        max_starts_goals = ((w*h) - len(obstacles)) // 100 * 2  # 2% of the free locations are start or goal locations

        start_locations = random_start_or_goal_locations(obstacles, free_locations, max_starts_goals//2)
        goal_locations = random_start_or_goal_locations(obstacles, free_locations, max_starts_goals//2)
        intersection_locations = list(set(start_locations) & set(goal_locations))
        union_locations = start_locations + goal_locations

        for loc in intersection_locations:
            if random.randrange(0, 30) == 0:
                # Remove the intersection with a 30% chance, so that the start and goal locations are not completely the same.
                if random.random() < 0.5:
                    start_locations.remove(loc)
                else:
                    goal_locations.remove(loc)

        free_locations = [x for x in free_locations if tuple(x) not in union_locations]
        agents_locations = random_agents_locations(free_locations, 80, union_locations + obstacles)
        agents = []
        locs = []
        i = 0
        for loc in agents_locations:
            string = "agent" + str(i)
            agents.append({'start': tuple([loc[0], loc[1]]), 'name': string})
            locs.append(tuple((loc[0], loc[1])))
            i = i + 1
        yaml_dic['agents'] = agents
        yaml_dic['map'] = {'dimensions': [w, h], 'obstacles': obstacles, 'non_task_endpoints': locs,
                           'start_locations': start_locations, 'goal_locations': goal_locations}

    with open(os.path.join(RootPath.get_root(), 'config.json'), 'r') as json_file:
        config = json.load(json_file)
    with open(os.path.join(os.path.join(RootPath.get_root(), config['input_path']), map_name + '.yaml'),
              'w') as param_file:
        yaml.dump(yaml_dic, param_file)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-map_name', help='The name of the map in the Benchmarks folder to convert', default="",
                        type=str)

    args = parser.parse_args()

    if args.map_name != "":
        map_converter(args.map_name)
        exit(0)

    for map in os.listdir(os.path.join(RootPath.get_root() + '/Benchmarks')):
        if map.endswith(".map"):
            map_name = map[:-4]
            map_converter(map_name)
