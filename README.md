# Traversability Mapping and Motion Planning
This is a repository for traversability mapping for UGVs in ROS. The code is modified on the repository [Traversability Mapping](https://github.com/TixiaoShan/traversability_mapping).


## Get Started

- Install [ROS](http://www.ros.org/install/).

- Install [ROS Navigation stack](http://wiki.ros.org/navigation). You can install it by running ```sudo apt-get install ros-version-navigation```. If you are using other versions of ROS, replace indigo in the command with your ROS version.


## Run the System (in simulation)

1. Run the launch file:
```
roslaunch traversability_mapping offline.launch
```
or
```
roslaunch traversability_mapping dyx.launch
```
and set ```use_sim_time``` as ```true```

2. Play existing bag files:
```
rosbag play *.bag --clock --topic /velodyne_points
```

## Run the System (with real robot)

Run the launch file:
<!-- ```
roslaunch traversability_mapping online.launch
``` -->
```
roslaunch traversability_mapping dyx.launch
```
and set ```use_sim_time``` as ```false```

<!-- ## Cite *Traversability_Mapping*

Thank you for citing our paper if you use any of this code: 
```
@inproceedings{bayesian2018shan,
  title={Bayesian Generalized Kernel Inference for Terrain Traversability Mapping},
  author={Shan, Tixiao and Wang, Jinkun and Englot, Brendan and Doherty, Kevin},
  booktitle={In Proceedings of the 2nd Annual Conference on Robot Learning},
  year={2018}
}
``` -->
