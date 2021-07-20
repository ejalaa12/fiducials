## fiducial_slam fiducial_slam_node.py

This node performs 3D Simultaneous Localization and Mapping (SLAM) from the 
fiducial transforms. For the mapping part, pairs of transforms are combined
to determine the position of fiducials based on existing observations.
For the localization part, fiducial transforms are combined with fiducial poses
to estimate the camera pose (and hence the robot pose).

Documentation is [on the ROS wiki](http://wiki.ros.org/fiducial_slam).


## How it works

- For each fiducial_transforms observation set
    - compute map -> base = map2fid(map) + fid2cam(obs) + cam2base(tf)
- average all map -> base
- compute map2cam = map2base(average) + base2cam(tf)
- Use the averaged map->cam to update the pose of each fiducial in map using a kalman observation step

## How I want to tweak it

### Step 1 Build the map

use fiducial_slam to build the map


### Step 2 Use the map

- load the map
- for each fiducial_transforms observation set
    - compute struct2map = struct2fid(map) + fid2cam(obs) + cam2base(tf) + base2map(tf)
- average all struct -> map
- publish map -> struct

