#ifndef MAP_CONVERSION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define MAP_CONVERSION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_

#include "map_conversion/cloud_filter/cloud_filter_interface.hpp"

namespace map_conversion {
class NoFilter: public CloudFilterInterface {
  public:
    NoFilter();

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

};
}
#endif