/*该类的功能主要是点云的二维栅格处理
 edited by goldqiu -2022-04-9
*/
#ifndef MAP_CONVERSION_POINTCLOUD_2D_PROCESS_HPP_
#define MAP_CONVERSION_POINTCLOUD_2D_PROCESS_HPP_
#include "map_conversion/ros_topic_interface/cloud_data.hpp"
#include <yaml-cpp/yaml.h>
#include "map_conversion/utility.hpp"
#include "map_conversion/cloud_filter/cloud_filter_interface.hpp"
#include "map_conversion/cloud_filter/voxel_filter.hpp"

namespace map_conversion {
class Pointcloud2dProcess {
  public:
    Pointcloud2dProcess(YAML::Node& config_node);
    Pointcloud2dProcess() = default;

    void find_Z_value(CloudData::CLOUD_PTR cloud_data); //计算点云中Z轴的最大和最小值
    int global_map_init(); //初始化全局地图
    //直通滤波器
    void PassThroughFilter(CloudData::CLOUD_PTR  pcd_cloud,CloudData& cloud_after_PassThrough, const bool &flag_in,double z_max, double z_min);
    //将三维点云转换为三维栅格
    void Pointcloud_to_2d_grid(const CloudData& pcd_cloud, nav_msgs::OccupancyGrid& msg, double map_resolution);

    double max_z;
    double min_z;
    std::string Global_map_file;
    std::string local_cloud_topic;
    std::string topic_frame_id;

    CloudData::CLOUD_PTR global_map_data;
    CloudData global_map_after_filter;
    CloudData global_map_no_filter;

    double grid_x = 0.1;
    double grid_y = 0.1;
    double grid_z = 0.1;

    double global_map_resolution = 1;
    double current_map_resolution = 0.1;

    double thre_radius = 0.5; 

    double global_max_z_adjust;
    double global_min_z_adjust;
    double current_max_z_adjust;
    double current_min_z_adjust;
  private:
      std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

  private:

};
}

#endif