
## Details
This node subscribes to `/points_raw`, `/points_map` and `/current_pose` and crops the original point cloud map with respect to the location of the ego-vehicle. This is published under `/reduced_map`. While `/points_raw` serves as a trigger for cropping the original point cloud map from `/points_map`, `/current_pose` continuously updates the ego-vehicle's pose and orientation to ensure that the cropping operations occur with respect to the latest vehicle pose. Since these operations are relatively expensive, the trigger callback (`/points_raw`) keeps a short buffer to make sure the data is still relevant.

## Setup

1. Create a ROS workspace and place this node within the `src` directory.  
2. Run `catkin_make --pkg map_reduction` if multiple packages are present within the workspace.
3. Please ensure that the package dependencies are met if running outside Docker: `pcl_ros`, `pcl_conversions`.

The code associated with this repository corresponds to the following research work. If you find it useful for research, please consider citing our work.
```
@inproceedings{paz20:mapping,
 address = {Las Vegas, NV},
 author = {David Paz and Hengyuan Zhang and Qinru Li and Hao Xiang and Henrik I Christensen},
 booktitle = {International Conference on Intelligent Robots and Systems (IROS)},
 month = {Oct},
 organization = {IEEE/RSJ},
 pages = { },
 title = {Probabilistic Semantic Mapping for Urban Autonomous Driving Applications},
 year = {2020}
}
```
