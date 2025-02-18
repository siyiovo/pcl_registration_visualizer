# pcl_registration_visualizer

![image](https://github.com/siyiovo/pcl_registration_visualizer/blob/main/assets/visualization.png)

## Installation

### Dependencies

> 1. CMake >= 3.0.2
> 2. Eigen3 >= 3.4
> 3. PCL >= 1.10
> 4. [KISS_Matcher](https://github.com/MIT-SPARK/KISS-Matcher) (Optional, TBD)

### Compilation

```
cd $YOUR_OWN_WORKSPACE$/src
git clone https://github.com/siyiovo/pcl_registration_visualizer.git
catkin build
```

please use your own workspace instead of YOUR_OWN_WORKSPACE.

### Using 

I code an example which use pcl::IterativeClosestPoint as pointcloud registration algorithm, you can run it by:

```
cd $YOUR_OWN_WORKSPACE$
source ./devel/setup.bash
roslaunch pcl_registration_visualizer icp_registration.launch
```

**NOTE: **you can press `n` to get next iteration result on visualizer.

### Parameters

some key parameters explanation here:

`/no_priori_pcd`

do not use priori pcd file in folder `pcd/`, and generate a random pointcloud. default: `true`

`/keypoint_mode`

if the number of points > `30000`, it will getKeypoint and visualize. default: `false`

`/max_iter_time`

maximum iteration time, which also the time you press `n`. default: `20`

`/getKeypoint/neighbor_num`

the number of nearest points. default: `10`

`/getKeypoint/curvature_threshold`

the threshold of curvature, here we get points that high curvature, which aslo the corner point. default: `0.10`

`/hasConverged/min_singular_threshold`

the threshold of minimum singular value. default: `1e-5`

### TODO

1. support [KISS_Matcher](https://github.com/MIT-SPARK/KISS-Matcher)
2. support ROS2 port
3. code an example of full pointcloud registration, which include coarse registration and fine registration
4. overload the visualization method



