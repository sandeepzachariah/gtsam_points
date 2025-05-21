#pragma once
#ifdef GTSAM_POINTS_USE_INTENSITY          // set from CMake later
  #include <pcl/point_types.h>
  using PointT = pcl::PointXYZI;
#else
  #include <pcl/point_types.h>
  using PointT = pcl::PointXYZ;
#endif