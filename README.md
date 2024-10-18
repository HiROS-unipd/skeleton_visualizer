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


## Citation
Please cite the following paper:
```
Guidolin, M., Tagliapietra, L., Menegatti, E., & Reggiani, M. (2023). Hi-ROS: Open-source multi-camera sensor fusion for real-time people tracking. Computer Vision and Image Understanding, 232, 103694.
```

Bib citation source:
```bibtex
@article{GUIDOLIN2023103694,
  title = {Hi-ROS: Open-source multi-camera sensor fusion for real-time people tracking},
  journal = {Computer Vision and Image Understanding},
  volume = {232},
  pages = {103694},
  year = {2023},
  issn = {1077-3142},
  doi = {https://doi.org/10.1016/j.cviu.2023.103694},
  url = {https://www.sciencedirect.com/science/article/pii/S1077314223000747},
  author = {Mattia Guidolin and Luca Tagliapietra and Emanuele Menegatti and Monica Reggiani},
  keywords = {Markerless motion capture, Multi-view body tracking, Real-time, ROS}
}
```
