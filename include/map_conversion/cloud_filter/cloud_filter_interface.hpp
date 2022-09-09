#ifndef MAP_CONVERSION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define MAP_CONVERSION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "map_conversion/ros_topic_interface/cloud_data.hpp"

namespace map_conversion {
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;

};
}

#endif