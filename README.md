
## Details
This node subscribes to `/points_raw`, `/points_map` and `/current_pose` and crops the original point cloud map with respect to the location of the ego-vehicle. This is published under `/reduced_map`. While `/points_raw` serves as a trigger for cropping the original point cloud map from `/points_map`, `/current_pose` continuously updates the ego-vehicle's pose and orientation to ensure that the cropping operations occur with respect to the latest vehicle pose. Since these operations are relatively expensive, the trigger callback (`/points_raw`) keeps a short buffer to make sure the data is still relevant.

## Setup

1. Create a ROS workspace and place this node within the `src` directory.  
2. Run `catkin_make --pkg map_reduction` if multiple packages are present within the workspace.
3. If running on an AVL Docker container, please ensure that the package dependencies are met: `pcl_ros`, `pcl_conversions`.
