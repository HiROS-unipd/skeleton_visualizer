# Hi-ROS Skeleton Visualizer

This ROS package allows to visualize a SkeletonGroup in RViz.


## Dependencies
* [Hi-ROS Skeleton Messages](https://github.com/hiros-unipd/skeleton_msgs)


## Launch files
**default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice


## Parameters

| Parameter                    | Description                                                                   |
| ---------------------------- | ----------------------------------------------------------------------------- |
| `node_required`              | Set if the other ROS nodes on the PC should be killed when the node is killed |
| `node_name`                  | Node name                                                                     |
| `seed`                       | Seed to use to generate each skeleton's color                                 |
| `lifetime`                   | Lifetime [s] of the visualized skeletons                                      |
| `alpha`                      | Alpha of the visualized skeletons                                             |
| `input_skeleton_group_topic` | Name of the input SkeletonGroup topic                                         |
| `output_marker_array_topic`  | Name of the output MarkerArray topic                                          |


## Usage
```
roslaunch hiros_skeleton_visualizer custom_configuration_example.launch
```
