# ViBe Algorithm for Colour or Monochrome Image Sequence

## Dependencies
- ROS
- OpenCV


## Compilation
``` bash
cd <catkin_ws> # navigate to catkin workspace
rosdep install --from-paths src --ignore-src -r -y # install all dependencies specified in
catkin build vibe_ros
```

## Topics

To change the topics to subscribe from, it is located in `vibe_ros.hpp` constructor.
