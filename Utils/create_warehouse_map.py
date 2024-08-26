import os
import yaml
import json
import RootPath
from visualize_map import showMap
from tkinter import Tk, Label, Entry, Button

class WarehouseParams:
    def __init__(self):
        self.map_name = ""
        self.shelves = 0
        self.pickup_per_shelf = 0

    def submit_values(self, frame, map_name_entry, shelves_entry, pickup_per_shelf_entry):
        self.map_name = map_name_entry.get()
        self.shelves = int(shelves_entry.get())
        self.pickup_per_shelf = int(pickup_per_shelf_entry.get())
        frame.destroy()


if __name__ == '__main__':
    frame = Tk()
    frame.title("Warehouse Data Input")
    frame.resizable(False, False)

    label = Label(frame, text='Map Name: ')
    label.grid(row=0, column=0)

    map_name_input_text = Entry(frame, width=20)
    map_name_input_text.grid(row=0, column=1)

    label = Label(frame, text='Shelves number: ')
    label.grid(row=1, column=0)

    shelves_input_text = Entry(frame, width=20)
    shelves_input_text.grid(row=1, column=1)

    label = Label(frame, text='Pickup Per Shelf Number: ')
    label.grid(row=2, column=0)

    pickup_per_shelf_input_text = Entry(frame, width=20)
    pickup_per_shelf_input_text.grid(row=2, column=1)

    params = WarehouseParams()

    frame.bind('<Return>', lambda event=None: params.submit_values(frame, map_name_input_text, shelves_input_text, pickup_per_shelf_input_text))

    button = Button(frame, text="Submit", command=lambda: params.submit_values(frame, map_name_input_text, shelves_input_text, pickup_per_shelf_input_text))
    button.grid(row=3, column=1)
    frame.mainloop()

    map_name = params.map_name
    shelves = params.shelves
    pickup_per_shelf = params.pickup_per_shelf
    horizontal_spaces = 3
    shelf_height = 3

    height = 2 + shelf_height * shelves + shelves - 1
    width = horizontal_spaces * 2 + pickup_per_shelf * 2 + 1

    yamlMap = {'agents': [], 'map': {}}
    yamlMap['map']['dimensions'] = [width, height]
    yamlMap['map']['start_locations'] = []
    yamlMap['map']['goal_locations'] = []
    yamlMap['map']['obstacles'] = []
    yamlMap['map']['non_task_endpoints'] = []

    for y in range(height):
        for x in range(width):
            location = (x, y)
            if (x == 0 or x == width - 1) and y % 2 == 0:
                yamlMap['map']['goal_locations'].append(tuple([x, y]))
            elif (x == 0 or x == width - 1 or x == 2 or x == width - 3) and y % 2 == 1:
                yamlMap['agents'].append({'start': tuple([x, y]), 'name': 'agent' + str(len(yamlMap['agents']))})
                yamlMap['map']['non_task_endpoints'].append(tuple([x, y]))
            elif horizontal_spaces - 1 < x < width - horizontal_spaces:
                if y % 4 == 2:
                    yamlMap['map']['obstacles'].append(tuple([x, y]))
                elif y % 4 == 1 or y % 4 == 3:
                    if x % 2 == 0:
                        yamlMap['map']['start_locations'].append(tuple([x, y]))
                    else:
                        yamlMap['map']['obstacles'].append(tuple([x, y]))

    with open(os.path.join(RootPath.get_root(), 'config.json'), 'r') as json_file:
        config = json.load(json_file)
    with open(os.path.join(os.path.join(RootPath.get_root(), config['input_path']), map_name + '.yaml'),
              'w') as yamlFile:
        yaml.dump(yamlMap, yamlFile)

    showMap(yamlMap, map_name, legend=True)

