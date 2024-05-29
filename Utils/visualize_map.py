import matplotlib
from matplotlib.patches import Circle, Rectangle, RegularPolygon
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import os
import RootPath
import yaml

Colors = ['orange', 'blue', 'green']


def showMap(map):
    map = map["map"]
    aspect = map["dimensions"][0] / map["dimensions"][1]

    fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
    ax = fig.add_subplot(111, aspect='equal')
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

    patches = []
    artists = []
    # Create boundary patch
    xmin = -0.5
    ymin = -0.5
    xmax = map["dimensions"][0] - 0.5
    ymax = map["dimensions"][1] - 0.5

    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)

    patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))

    for x in range(map["dimensions"][0]):
        for y in range(map["dimensions"][1]):
            patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='none', edgecolor='black'))

    for o in map["obstacles"]:
        x, y = o[0], o[1]
        patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='black', edgecolor='black'))

    for e in map["non_task_endpoints"]:
        x, y = e[0], e[1]
        patches.append(Circle((x, y), 0.4, facecolor='green', edgecolor='black'))

    for s in map["start_locations"]:
        patches.append(
            Rectangle((s[0] - 0.5, s[1] - 0.5), 1, 1, facecolor=Colors[1],
                      edgecolor='black',
                      alpha=0.5))
        startText = ax.text(s[0], s[1], "S")
        startText.set_horizontalalignment('center')
        startText.set_verticalalignment('center')
        artists.append(startText)
    
    for g in map["goal_locations"]:
        patches.append(
            Rectangle((g[0] - 0.5, g[1] - 0.5), 1, 1, facecolor=Colors[0],
                      edgecolor='black',
                      alpha=0.5))
        goalText = ax.text(g[0], g[1], "G")
        goalText.set_horizontalalignment('center')
        goalText.set_verticalalignment('center')
        artists.append(goalText)

    for p in patches:
        ax.add_patch(p)

    for a in artists:
        ax.add_artist(a)
    plt.show()


if __name__ == '__main__':
    map_name = "input_warehouse_big_random_with_middle_corridors.yaml"
    map_path = os.path.join(RootPath.get_root(), os.path.join("Environments", map_name, ))

    with open(map_path, 'r') as map_file:
        try:
            map = yaml.load(map_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    map_path_tmp = os.path.join(RootPath.get_root(), os.path.join("Environments", map_name + "_tmp", ))

    with open(map_path_tmp, 'w') as map_file:
        yaml.safe_dump(map, map_file)

    with open(map_path_tmp) as map_file:
        map = yaml.load(map_file, Loader=yaml.FullLoader)

    showMap(map)
