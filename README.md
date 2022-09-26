# Hi-ROS Skeleton Visualizer

This ROS package allows to visualize a SkeletonGroup in RViz.


## Dependencies
* [Hi-ROS Skeleton Messages](https://github.com/hiros-unipd/skeleton_msgs)


## Parameters

| Parameter                    | Description                                                                   |
| ---------------------------- | ----------------------------------------------------------------------------- |
| `seed`                       | Seed to use to generate each skeleton's color                                 |
| `lifetime`                   | Lifetime [s] of the visualized skeletons                                      |
| `alpha`                      | Alpha of the visualized skeletons                                             |
| `input_topic`                | Name of the input SkeletonGroup topic                                         |
| `output_topic`               | Name of the output MarkerArray topic                                          |


## Usage
```
ros2 launch hiros_skeleton_visualizer default.launch.py
```
