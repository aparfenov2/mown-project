#include <parameter_assertions/assertions.h>

#include "unrolling_layer_config.h"

namespace unrolling_layer
{
UnrollingLayerConfig::UnrollingLayerConfig(const ros::NodeHandle &parent_nh, std::string name)
{
  ros::NodeHandle nh{ parent_nh, name };

  assertions::getParam(nh, "topic", topic);
}
}  // namespace unrolling_layer
