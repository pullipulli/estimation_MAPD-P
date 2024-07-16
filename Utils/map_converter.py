"""
This script is used to convert ASCII maps to YAML format.
"""
import argparse
import json
import os
import random
import re
import yaml
import RootPath


def is_valid_agent_location(locations, agent_location):
    if agent_location in locations:
        return False

    for location in locations:
        if abs(location[0] - agent_location[0]) <= 2 and abs(location[1] - agent_location[1]) <= 2:
            return False

    return True


def random_agents_locations(free_locations, num_agents):
    free_locs = list(free_locations)
    agent_locations = []

    for i in range(num_agents):
        agent_location = random.choice(free_locs)
        free_locs.remove(agent_location)

        if is_valid_agent_location(agent_locations, agent_location):
            agent_locations.append(agent_location)
        else:
            i -= 1

    return agent_locations


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
        start_locations = []
        goal_locations = []
        agents_locations = []
        for i in range(h - 1, -1, -1):
            line = ascii_map.readline()
            print(line)
            for j in range(w):
                if line[j] == '@' or line[j] == 'T':
                    yaml_dic['map']['obstacles'].append((j, i))
                else:
                    free_locations.append([j, i])
        start_locations = random.sample(free_locations, 50)

        free_locations = [x for x in free_locations if x not in start_locations]
        goal_locations = random.sample(free_locations, 40)
        free_locations = [x for x in free_locations if x not in goal_locations]
        agents_locations = random_agents_locations(free_locations, 80)
        agents = []
        locs = []
        i = 0
        for loc in agents_locations:
            string = "agent" + str(i)
            agents.append({'start': [loc[0], loc[1]], 'name': string})
            locs.append((loc[0], loc[1]))
            i = i + 1
        yaml_dic['agents'] = agents
        yaml_dic['map'] = {'dimensions': [w, h], 'obstacles': yaml_dic['map']['obstacles'], 'non_task_endpoints': locs,
                           'start_locations': start_locations, 'goal_locations': goal_locations}

    with open(os.path.join(RootPath.get_root(), 'config.json'), 'r') as json_file:
        config = json.load(json_file)
    with open(os.path.join(os.path.join(RootPath.get_root(), config['input_path']), map_name + '.yaml'),
              'w') as param_file:
        yaml.dump(yaml_dic, param_file)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-map_name', help='The name of the map in the Benchmarks folder to convert', default="", type=str)

    args = parser.parse_args()

    map_converter(args.map_name)
