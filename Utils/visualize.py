"""
This file contains the code to visualize the simulation. It uses matplotlib to create an animation of the simulation.
The animation shows the agents moving in the environment and the tasks being completed.
"""

import matplotlib
from matplotlib.patches import Circle, Rectangle, RegularPolygon
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
matplotlib.use("TkAgg")

Colors = ['orange', 'blue', 'green']


class Animation:
    """
    This class creates an animation of the simulation. It shows the agents moving in the environment and the tasks being completed.
    Yellow Circles are used to represent the agents.
    Green Circles are used to represent the non task endpoints.
    Black rectangles are used to represent the obstacles.
    Orange/Blue/Green little squares are used to represent the pickup.
    Orange/Blue/Green Triangles are used to represent the goal.
    """
    def __init__(self, map, schedule, slow_factor=10):
        self.map = map
        self.schedule = schedule
        self.slow_factor = slow_factor
        self.combined_schedule = {}
        self.combined_schedule.update(self.schedule["schedule"])
        self.tasks = dict()

        aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        # Create boundary patch
        xmin = -0.5
        ymin = -0.5
        xmax = map["map"]["dimensions"][0] - 0.5
        ymax = map["map"]["dimensions"][1] - 0.5

        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)

        self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
        for x in range(map["map"]["dimensions"][0]):
            for y in range(map["map"]["dimensions"][1]):
                self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='none', edgecolor='black'))
        for o in map["map"]["obstacles"]:
            x, y = o[0], o[1]
            self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='black', edgecolor='black'))
        for e in map["map"]["non_task_endpoints"]:
            x, y = e[0], e[1]
            self.patches.append(Circle((x, y), 0.4, facecolor='green', edgecolor='black'))

        task_colors = np.random.rand(len(map['tasks']), 3)
        for t, i in zip(map["tasks"], range(len(map["tasks"]))):
            x_s, y_s = t['start'][0], t['start'][1]
            self.tasks[t['task_name']] = [
                Rectangle((x_s - 0.25, y_s - 0.25), 0.5, 0.5, facecolor=task_colors[i], edgecolor='black', alpha=0)]
            self.patches.append(self.tasks[t['task_name']][0])

        for t, i in zip(map["tasks"], range(len(map["tasks"]))):
            x_g, y_g = t['goal'][0], t['goal'][1]
            self.tasks[t['task_name']].append(
                RegularPolygon((x_g, y_g - 0.05), 3, radius=0.2, facecolor=task_colors[i], edgecolor='black', alpha=0))
            self.patches.append(self.tasks[t['task_name']][1])

        # Create agents:
        self.T = 0
        # Draw goals first
        for d, i in zip(map["agents"], range(0, len(map["agents"]))):
            if 'goal' in d:
                self.patches.append(
                    Rectangle((d["goal"][0] - 0.25, d["goal"][1] - 0.25), 0.5, 0.5, facecolor=Colors[0],
                              edgecolor='black',
                              alpha=0.5))
        for d, i in zip(map["agents"], range(0, len(map["agents"]))):
            name = d["name"]
            self.agents[name] = Circle((d["start"][0], d["start"][1]), 0.3, facecolor=Colors[0], edgecolor='black')
            self.agents[name].original_face_color = Colors[0]
            self.patches.append(self.agents[name])
            self.T = max(self.T, schedule["schedule"][name][-1]["t"])
            self.agent_names[name] = self.ax.text(d["start"][0], d["start"][1], name.replace('agent', ''))
            self.agent_names[name].set_horizontalalignment('center')
            self.agent_names[name].set_verticalalignment('center')
            self.artists.append(self.agent_names[name])

        self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                                            init_func=self.init_func,
                                            frames=int(self.T + 1) * self.slow_factor,
                                            interval=10,
                                            blit=True,
                                            repeat=False)

    def save(self, file_name: str, speed=2) -> None:
        """
        Save the animation to a GIF file.
        :param file_name: name of the file to save the animation to
        :param speed: speed of the animation
        :return: 
        """
        writer = animation.ImageMagickFileWriter(fps=10 * speed)
        self.anim.save(
            file_name,
            writer)

    @staticmethod
    def show():
        """Show the animation."""
        plt.show()

    def init_func(self):
        """
        Initialize the animation.
        :return:
        """
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, i):
        """
        Animate the simulation.
        :param i:
        :return:
        """
        for agent_name, agent in self.combined_schedule.items():
            pos = self.getState(i / self.slow_factor, agent)
            p = (pos[0], pos[1])
            self.agents[agent_name].center = p
            self.agent_names[agent_name].set_position(p)

        # Reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        # Make tasks visible at the right time
        for t in self.map['tasks']:
            if t['start_time'] <= i / self.slow_factor + 1 <= self.schedule['completed_tasks_times'][t['task_name']]:
                self.tasks[t['task_name']][0].set_alpha(0.5)
                self.tasks[t['task_name']][1].set_alpha(0.5)
            else:
                self.tasks[t['task_name']][0].set_alpha(0)
                self.tasks[t['task_name']][1].set_alpha(0)

        # Check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {})".format(i, j))

        return self.patches + self.artists

    def getState(self, t, d):
        """
        Get the state of the simulation at time t.
        :param t:
        :param d:
        :return:
        """
        idx = 0
        while idx < len(d) and d[idx]["t"] < t:
            idx += 1
        if idx == 0:
            return np.array([float(d[0]["x"]), float(d[0]["y"])])
        elif idx < len(d):
            posLast = np.array([float(d[idx - 1]["x"]), float(d[idx - 1]["y"])])
            posNext = np.array([float(d[idx]["x"]), float(d[idx]["y"])])
        else:
            return np.array([float(d[-1]["x"]), float(d[-1]["y"])])
        dt = d[idx]["t"] - d[idx - 1]["t"]
        t = (t - d[idx - 1]["t"]) / dt
        pos = (posNext - posLast) * t + posLast
        return pos
