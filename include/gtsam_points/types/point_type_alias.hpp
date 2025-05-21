#pragma once
#include <gtsam_points/config.hpp>
#include <pcl/point_types.h>

#ifdef __cplusplus
namespace gtsam_points {
#endif

#ifdef GTSAM_POINTS_USE_INTENSITY
using PointT = pcl::PointXYZI;
#else
using PointT = pcl::PointXYZ;
#endif

#ifdef __cplusplus
}  // namespace gtsam_points
#endif