/*该类的功能主要是点云的二维栅格处理
 edited by goldqiu -2022-04-9
*/
#include "map_conversion/pointcloud_process/pointcloud_2d_process.hpp"
#include "glog/logging.h"

namespace map_conversion {
Pointcloud2dProcess::Pointcloud2dProcess(YAML::Node& config_node)
: global_map_data(new CloudData::CLOUD())
{

   Global_map_file = config_node["Global_file_directory"].as<std::string>();
   local_cloud_topic = config_node["local_cloud_frame"].as<std::string>();
   topic_frame_id = config_node["frame_id"].as<std::string>();

   global_max_z_adjust = atof((config_node["global_max_z"].as<std::string>()).c_str());
   global_min_z_adjust = atof((config_node["global_min_z"].as<std::string>()).c_str());
   current_max_z_adjust = atof((config_node["current_max_z"].as<std::string>()).c_str());
   current_min_z_adjust = atof((config_node["current_min_z"].as<std::string>()).c_str());

   global_map_resolution = atof((config_node["2d_global_map_resolution"].as<std::string>()).c_str());
   current_map_resolution = atof((config_node["2d_current_map_resolution"].as<std::string>()).c_str());

  //  cout << "z_adjust:" << local_z_adjust << endl;
}

void Pointcloud2dProcess::find_Z_value(CloudData::CLOUD_PTR cloud_data){
    std::cout << "初始点云数据点数：" << cloud_data->points.size() << std::endl;

    for(int i = 0; i < cloud_data->points.size() - 1; i++){
        if(cloud_data->points[i].z>max_z){
            max_z=cloud_data->points[i].z;
          }
        if(cloud_data->points[i].z<min_z){
            min_z=cloud_data->points[i].z;
          }
    }
    std::cout<<"orig max_z="<<max_z<<",min_z="<<min_z<<std::endl;
}

int Pointcloud2dProcess::global_map_init(void)
{
    if (pcl::io::loadPCDFile(Global_map_file, *(global_map_data)) == -1)
  {
        PCL_ERROR ("Couldn't read file: %s \n", Global_map_file.c_str());
        return (-1);
  } 
    LOG(INFO) << "Load global map, size:" << global_map_data->points.size();

    // global_map_filter_ptr_ = std::make_shared<VoxelFilter>(0.2,0.2,0.2);

    // global_map_filter_ptr_->Filter(global_map_data, global_map_data);
    
    // LOG(INFO) << "Filtered global map, size:" << global_map_data->points.size();

    find_Z_value(global_map_data);
    
    PassThroughFilter(global_map_data,global_map_no_filter,false,0,0);

    PassThroughFilter(global_map_data,global_map_after_filter,false,global_max_z_adjust,global_min_z_adjust);
}

void Pointcloud2dProcess::PassThroughFilter(CloudData::CLOUD_PTR  pcd_cloud,CloudData& cloud_after_PassThrough, const bool &flag_in,double z_max, double z_min)
{
    /*方法一：直通滤波器对点云进行处理。*/
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcd_cloud);//输入点云
    passthrough.setFilterFieldName("z");//对z轴进行操作
    passthrough.setFilterLimits(min_z+z_min, max_z-z_max);//设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(flag_in);//true表示保留范围外，false表示保留范围内
    passthrough.filter(*cloud_after_PassThrough.cloud_ptr);//执行滤波，过滤结果保存在 cloud_after_PassThrough
    std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough.cloud_ptr->points.size() << std::endl;
    // for(int i = 0; i < cloud_after_PassThrough.cloud_ptr->points.size() - 1; i++){
    //  if(cloud_after_PassThrough.cloud_ptr->points[i].z>max_z){
    //    max_z=cloud_after_PassThrough.cloud_ptr->points[i].z;
    //  }
    //  if(cloud_after_PassThrough.cloud_ptr->points[i].z<min_z){
    //    min_z=cloud_after_PassThrough.cloud_ptr->points[i].z;
    //  }
    //  }
    // std::cout<<"aft pass through filter: max_z="<<max_z<<",min_z="<<min_z<<std::endl;
}

void Pointcloud2dProcess::Pointcloud_to_2d_grid(const CloudData& pcd_cloud, nav_msgs::OccupancyGrid& msg, double map_resolution)
{
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = map_resolution;

  double x_min, x_max, y_min, y_max;
  double z_max_grey_rate = 0.05;
  double z_min_grey_rate = 0.95;
  double k_line = (z_max_grey_rate - z_min_grey_rate) / (max_z - min_z);
  double b_line = (max_z * z_min_grey_rate - min_z * z_max_grey_rate) / (max_z - min_z);

  if(pcd_cloud.cloud_ptr->points.empty())
  {
    ROS_WARN("pcd is empty!\n");
    return;
  }

  for(int i = 0; i < pcd_cloud.cloud_ptr->points.size() - 1; i++)
  {
    if(i == 0)
    {
      x_min = x_max = pcd_cloud.cloud_ptr->points[i].x;
      y_min = y_max = pcd_cloud.cloud_ptr->points[i].y;
    }

    double x = pcd_cloud.cloud_ptr->points[i].x;
    double y = pcd_cloud.cloud_ptr->points[i].y;

    if(x < x_min) x_min = x;
    if(x > x_max) x_max = x;

    if(y < y_min) y_min = y;
    if(y > y_max) y_max = y;
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = int((x_max - x_min) / map_resolution);
  msg.info.height = int((y_max - y_min) / map_resolution);

  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  ROS_INFO("data size = %d\n", msg.data.size());

  for(int iter = 0; iter < pcd_cloud.cloud_ptr->points.size(); iter++)
  {
    int i = int((pcd_cloud.cloud_ptr->points[iter].x - x_min) / map_resolution);
    if(i < 0 || i >= msg.info.width) continue;

    int j = int((pcd_cloud.cloud_ptr->points[iter].y - y_min) / map_resolution);
    if(j < 0 || j >= msg.info.height - 1) continue;

    msg.data[i + j * msg.info.width] = 100;
//    msg.data[i + j * msg.info.width] = int(255 * (pcd_cloud.cloud_ptr->points[iter].z * k_line + b_line)) % 255;
  }
}
} // namespace data_input