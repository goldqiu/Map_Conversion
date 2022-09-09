# Map_Conversion
导航“前端”，将定位后的三维点云实时或离线三维到二维栅格化，并计算代价生成代价地图。

### 运行

```
roslaunch map_conversion slam_to_planning.launch
```

### 效果

![pic1](.\PIC\pic1.png)

![pic2](.\PIC\pic2.png)

### 参数

```
%YAML:1.0

Global_file_directory: "/home/qjs/code/ROS_Localization/global_localization_chapter4_ws/src/lidar_localization/slam_data/map/filtered_map2.pcd"   #全局地图文件位置
frame_id: "map"
local_cloud_frame:  "/current_scan"     #局部雷达原始帧话题 (/laser_cloud_map)

global_max_z: "0"       #这两个参数是调整全局栅格地图的Z轴直通滤波的范围
global_min_z: "35"
current_max_z: "5"       #这两个参数是调整实时栅格地图的Z轴直通滤波的范围
current_min_z: "0"

2d_global_map_resolution: "1"  
2d_current_map_resolution: "0.1"

3d_x_size: 30.0
3d_y_size: 30.0
3d_z_size: 10.0
```

### 文件介绍

```
│  CMakeLists.txt	
│  package.xml
│  README.md
│  
├─cmake
│      eigen.cmake
│      global_defination.cmake
│      glog.cmake
│      PCL.cmake
│      yaml.cmake
│      
├─config
│      params.yaml   //参数配置
│      rviz_test.rviz
│      
├─include
│  └─map_conversion
│      │  tic_toc.h   //运行时间计算类
│      │  utility.hpp //通用头文件、结构体等存放
│      │  
│      ├─cloud_filter  //点云滤波算法
│      │      box_filter.hpp
│      │      cloud_filter_interface.hpp
│      │      no_filter.hpp
│      │      voxel_filter.hpp
│      │      
│      ├─global_defination  //工程全局路径生成
│      │      global_defination.h.in
│      │      
│      ├─pointcloud_process  //点云处理类
│      │      costmap_calculator.hpp  //生成代价地图、包括考虑2.5d地形和障碍物
│      │      pointcloud_2d_process.hpp	//二维栅格化
│      │      pointcloud_3d_process.hpp //三维栅格化
│      │      
│      └─ros_topic_interface  //ros数据输入输出接口
│              cloud_data.hpp
│              cloud_publisher.hpp
│              cloud_subscriber.hpp
│              
├─launch
│      slam_to_planning.launch 
│      
├─PIC
│      pic1.png
│      pic2.png
│      
└─src
    ├─app  目前有两个节点
    │      global_submap_node.cpp   //全局子地图节点，10s的周期
    │      local_environment_node.cpp  //局部环境节点，用于局部规划，包括全局地图和局部地图的对齐，局部地图、局部代价地图										计算和生成等，10hz频率
    │      
    ├─cloud_filter
    │      box_filter.cpp
    │      no_filter.cpp
    │      voxel_filter.cpp
    │      
    ├─pointcloud_process  //点云处理类
    │      costmap_calculator.cpp  //生成代价地图，考虑地形和障碍物
    │      pointcloud_2d_process.cpp  //二维栅格化
    │      pointcloud_3d_process.cpp  //三维栅格化
    │      
    └─ros_topic_interface  //ros数据输入输出接口
            cloud_publisher.cpp
            cloud_subscriber.cpp
```

### 输入输出

输入：

全局点云pcd：global_map

当前雷达帧local_cloud_frame:  "/current_scan" 

输出：

全局三维点云地图、三维栅格地图、二维栅格地图

实时当前帧三维栅格地图、二维栅格地图