# Creation and implementation of an algorithm for MAPD with online estimation of task probability distribution

This is the source code of my thesis for the Bachelor's Degree in Computer Science at the University of Milan

This thesis is about integrating the online estimation of the task probability distribution in TP-M$_p$.

## Code structure

The code is structured as follows:
- 'Benchmarks' contains the map files of the benchmarks used in the experiments taken from the [Moving AI Lab](https://movingai.com/benchmarks/)
- 'Environments' contains the YAML files representing the environments used in the experiments (the benchmarks are converted to YAML files)
- 'Simulation' contains the code for the simulation and the implementation of the various versions of Token Pasing.
- 'Utils' contains various code utilities for the Experiments, the maps, types, the visualization of the results and of the maps, etc.
- 'demo.py' is a script that runs a demo of the simulation, but it is probably outdated and not working.
- 'RootPath.py' is a script that sets the root path of the project, so that the other scripts can import the modules correctly.

## How to run the Experiments

In the 'Utils/ShowStats' folder there are the scripts to run the experiments and show the relative results; 
'run_parallel_experiments.py' is the script that runs the experiments in parallel, while 'show_stats.py' is the script that shows the results or save them into the project directories.

After running the experiments (saved in .json in the ResultsJsons folder), you can use the 'show_stats.py' script to show the results.

## Convert from .map to .yaml

To convert a .map file to a .yaml file with the right format, you can use the 'map_converter.py' script in the 'Utils' folder.

## Creating a new warehouse environment

To create a new warehouse environment, you can use the 'create_warehouse_map.py' script in the 'Utils' folder.

It will create a new warehouse map with the specified parameters and save it in the 'Environments' folder.

## Visualization

To visualize the results of the experiments, you can use the 'show_stats.py' script in the 'Utils/ShowStats' folder.

To visualize the maps, you can use the 'visualize_map.py' script in the 'Utils/ShowStats' folder.

To visualize an entire simulation, you can use the 'visualize.py' script in the 'Utils/ShowStats' folder.

### Other things to note

The code in not completely documented, and probably not updated with the last version of the code.
The same applies to the type checking.

#### Acknowledgments

I would like to thank my supervisor, [Prof. Nicola Basilico](https://github.com/nbasilico), for his guidance and support during the development of this thesis.

Also, I would like to thank the previous students who worked on this project, as their code was the starting point of my thesis.

This code is available on GitHub here: 
- [Lodz97's Code; used to visualize the entire simulation and to integrate the deadlock recovery](https://github.com/Lodz97/Multi-Agent_Pickup_and_Delivery)
- [andrea4dipietro4's Code; used as a basis for my code](https://github.com/andrea4dipietro/MAPD-P)
