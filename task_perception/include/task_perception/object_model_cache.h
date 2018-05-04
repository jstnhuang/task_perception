#ifndef _PBI_OBJECT_MODEL_CACHE_H_
#define _PBI_OBJECT_MODEL_CACHE_H_

#include <map>
#include <string>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace pbi {
struct ObjectModelCache {
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  std::map<std::string, bool> is_circular;
};
}  // namespace pbi

#endif  // _PBI_OBJECT_MODEL_CACHE_H_
