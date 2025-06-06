// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <vector>
#include <random>
#include <Eigen/Core>

#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/point_type_alias.hpp>
#include <pcl/point_cloud.h>

struct CUstream_st;

namespace gtsam_points {

/**
 * @brief Point cloud frame on CPU memory
 */
struct PointCloudCPU : public PointCloud {
public:
  using Ptr = std::shared_ptr<PointCloudCPU>;
  using ConstPtr = std::shared_ptr<const PointCloudCPU>;

  PointCloudCPU();
  ~PointCloudCPU();

  /**
   * @brief Constructor
   * @param points     Pointer to point data
   * @param num_points Number of points
   */
  template <typename T, int D>
  PointCloudCPU(const Eigen::Matrix<T, D, 1>* points, int num_points);

  /**
   * @brief Constructor
   * @param points  Points
   */
  template <typename T, int D, typename Alloc>
  PointCloudCPU(const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& points) : PointCloudCPU(points.data(), points.size()) {}

  // Forbid shallow copy
  PointCloudCPU(const PointCloudCPU& points) = delete;
  PointCloudCPU& operator=(PointCloudCPU const&) = delete;

  /// Deep copy
  static PointCloudCPU::Ptr clone(const PointCloud& points);

  template <typename T>
  void add_times(const T* times, int num_points);
  template <typename T>
  void add_times(const std::vector<T>& times) {
    add_times(times.data(), times.size());
  }

  template <typename T, int D>
  void add_points(const Eigen::Matrix<T, D, 1>* points, int num_points);
  template <typename T, int D, typename Alloc>
  void add_points(const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& points) {
    add_points(points.data(), points.size());
  }

  template <typename T, int D>
  void add_normals(const Eigen::Matrix<T, D, 1>* normals, int num_points);
  template <typename T, int D, typename Alloc>
  void add_normals(const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& normals) {
    add_normals(normals.data(), normals.size());
  }

  template <typename T, int D>
  void add_covs(const Eigen::Matrix<T, D, D>* covs, int num_points);
  template <typename T, int D, typename Alloc>
  void add_covs(const std::vector<Eigen::Matrix<T, D, D>, Alloc>& covs) {
    add_covs(covs.data(), covs.size());
  }

  template <typename T>
  void add_intensities(const T* intensities, int num_points);
  template <typename T>
  void add_intensities(const std::vector<T>& intensities) {
    add_intensities(intensities.data(), intensities.size());
  }

  template <typename T>
  void add_aux_attribute(const std::string& attrib_name, const T* values, int num_points) {
    auto attributes = std::make_shared<std::vector<T>>(values, values + num_points);
    aux_attributes_storage[attrib_name] = attributes;
    aux_attributes[attrib_name] = std::make_pair(sizeof(T), attributes->data());
  }
  template <typename T, typename Alloc>
  void add_aux_attribute(const std::string& attrib_name, const std::vector<T, Alloc>& values) {
    add_aux_attribute(attrib_name, values.data(), values.size());
  }

  static PointCloudCPU::Ptr load(const std::string& path);

public:
  std::vector<double> times_storage;
  std::vector<Eigen::Vector4d> points_storage;
  std::vector<Eigen::Vector4d> normals_storage;
  std::vector<Eigen::Matrix4d> covs_storage;
  std::vector<double> intensities_storage;

  std::unordered_map<std::string, std::shared_ptr<void>> aux_attributes_storage;
};

/**
 * @brief Sample points by indices.
 * @param points   Input points
 * @param indices  Point indices
 * @return         Sampled points
 */
PointCloudCPU::Ptr sample(const PointCloud::ConstPtr& points, const std::vector<int>& indices);

/**
 * @brief Naive random sampling.
 * @param points         Input points
 * @param sampling_rate  Random sampling rate in [0, 1]
 * @param mt             RNG
 * @return               Downsampled points
 */
PointCloudCPU::Ptr random_sampling(const PointCloud::ConstPtr& points, const double sampling_rate, std::mt19937& mt);

/**
 * @brief Voxel grid downsampling.
 * @note  This algorithm takes the average of point attributes (whatever it is) of each voxel.
 *
 * @param points            Input points
 * @param voxel_resolution  Voxel resolution
 * @param num_threads       Number of threads
 * @return                  Downsampled points
 */
PointCloudCPU::Ptr voxelgrid_sampling(const PointCloud::ConstPtr& points, const double voxel_resolution, int num_threads = 1);

/**
 * @brief Voxel grid random sampling.
 * @note  This algorithm randomly samples points such that the number of sampled points of each voxel becomes (more or less) the same.
 *        This algorithm avoids mixing point attributes (unlike the standard voxelgrid downsampling), and thus can provide spatially
 *        well-distributed point samples with several attributes (e.g., normals and covs).
 *
 * @param points            Input points
 * @param voxel_resolution  Voxel resolution
 * @param sampling_rate     Random sampling rate in [0, 1]
 * @param mt                RNG
 * @param num_threads       Number of threads
 * @return                  Downsampled points
 */
PointCloudCPU::Ptr randomgrid_sampling(
  const PointCloud::ConstPtr& points,
  const double voxel_resolution,
  const double sampling_rate,
  std::mt19937& mt,
  int num_threads = 1);

/**
 * @brief Extract points for which pred returns true.
 * @param points  Input points
 * @param pred    Predicate function that takes Eigen::Vector4d and returns bool
 */
template <typename Func>
PointCloudCPU::Ptr filter(const PointCloud::ConstPtr& points, const Func& pred) {
  std::vector<int> indices;
  indices.reserve(points->size());
  for (int i = 0; i < points->size(); i++) {
    if (pred(points->points[i])) {
      indices.push_back(i);
    }
  }
  return sample(points, indices);
}

/**
 * @brief Extract points for which pred returns true.
 * @param points Input points
 * @param pred   Predicate function that takes a point index and returns bool
 */
template <typename Func>
PointCloudCPU::Ptr filter_by_index(const PointCloud::ConstPtr& points, const Func& pred) {
  std::vector<int> indices;
  indices.reserve(points->size());
  for (int i = 0; i < points->size(); i++) {
    if (pred(i)) {
      indices.push_back(i);
    }
  }
  return sample(points, indices);
}

/**
 * @brief Sort points
 * @param points Input points
 * @param pred   Comparison function that takes two point indices (lhs and rhs) and returns true if lhs < rhs
 */
template <typename Compare>
PointCloudCPU::Ptr sort(const PointCloud::ConstPtr& points, const Compare& comp) {
  std::vector<int> indices(points->size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), comp);
  return gtsam_points::sample(points, indices);
}

/**
 * @brief Sort points by time
 * @param points  Input points
 */
PointCloudCPU::Ptr sort_by_time(const PointCloud::ConstPtr& points);

/**
 * @brief Transform points, normals, and covariances
 *
 * @param points           Input points
 * @param transformation   Transformation
 * @return Transformed points
 */
template <typename Scalar, int Mode>
PointCloudCPU::Ptr transform(const PointCloud::ConstPtr& points, const Eigen::Transform<Scalar, 3, Mode>& transformation);

/**
 * @brief Transform points, normals, and covariances inplace
 * @param points           [in/out] Points to be transformed
 * @param transformation   Transformation
 */
template <typename Scalar, int Mode>
void transform_inplace(PointCloud& points, const Eigen::Transform<Scalar, 3, Mode>& transformation);

/**
 * @brief Transform points, normals, and covariances inplace
 * @param points           [in/out] Points to be transformed
 * @param transformation   Transformation
 */
template <typename Scalar, int Mode>
void transform_inplace(PointCloud::Ptr points, const Eigen::Transform<Scalar, 3, Mode>& transformation) {
  transform_inplace(*points, transformation);
}

/**
 * @brief Statistical outlier removal
 * @param points      Input points
 * @param neighbors   Neighbor indices
 * @param k           Number of neighbors (neighbors.size() must be >= points->size() * k)
 * @param std_thresh  Standard deviation multiplication threshold
 * @return            Inlier point indices
 */
std::vector<int>
find_inlier_points(const PointCloud::ConstPtr& points, const std::vector<int>& neighbors, const int k, const double std_thresh = 1.0);

/**
 * @brief Statistical outlier removal
 * @param points      Input points
 * @param neighbors   Neighbor indices
 * @param k           Number of neighbors (neighbors.size() must be >= points->size() * k)
 * @param std_thresh  Standard deviation multiplication threshold
 * @return            Filtered point cloud
 */
PointCloudCPU::Ptr remove_outliers(const PointCloud::ConstPtr& points, const std::vector<int>& neighbors, const int k, const double std_thresh = 1.0);

/**
 * @brief Statistical outlier removal
 * @param points      Input points
 * @param k           Number of neighbors (neighbors.size() must be >= points->size() * k)
 * @param std_thresh  Standard deviation multiplication threshold
 * @return            Filtered point cloud
 */
PointCloudCPU::Ptr remove_outliers(const PointCloud::ConstPtr& points, const int k = 10, const double std_thresh = 1.0, const int num_threads = 1);

/**
 * @brief Compute point distances
 * @param points          Input points
 * @param max_scan_count  Maximum number of points to scan
 * @return                Distances
 */
std::vector<double> distances(const PointCloud::ConstPtr& points, size_t max_scan_count = std::numeric_limits<size_t>::max());

/**
 * @brief Compute min and max point distances
 * @param points          Input points
 * @param max_scan_count  Maximum number of points to scan
 * @return                Min and max distances
 */
std::pair<double, double> minmax_distance(const PointCloud::ConstPtr& points, size_t max_scan_count = std::numeric_limits<size_t>::max());

/**
 * @brief Compute median point distance
 * @param points          Input points
 * @param max_scan_count  Maximum number of points to scan
 * @return                Median distance
 */
double median_distance(const PointCloud::ConstPtr& points, size_t max_scan_count = std::numeric_limits<size_t>::max());

/**
 * @brief Merge a set of frames into one frame
 * @note  This function only merges points and covs and discard other point attributes.
 * @param poses                  Poses of input frames
 * @param frames                 Input frames
 * @param downsample_resolution  Downsampling resolution
 * @return                       Merged frame
 */



// ---------------------------------------------------------------------------
// 1.  CPU helper — templated so we can carry PointXYZ  *or* PointXYZI
// ---------------------------------------------------------------------------
template <typename PointT = gtsam_points::PointT>
typename pcl::PointCloud<PointT>::Ptr
merge_frames(const std::vector<Eigen::Isometry3d>& poses,
             const std::vector<typename pcl::PointCloud<PointT>::ConstPtr>& frames,
             double downsample_resolution);

// ---------------------------------------------------------------------------
// 2.  GPU helper — KEEP THE ORIGINAL SIGNATURE (internal PointCloudGPU).
//                  We do **not** touch GPU code for intensity; the existing
//                  implementation still works.
// ---------------------------------------------------------------------------
PointCloud::Ptr
merge_frames_gpu(const std::vector<Eigen::Isometry3d>& poses,
                 const std::vector<PointCloud::ConstPtr>& frames,
                 double downsample_resolution,
                 CUstream_st* stream = 0);

// ---------------------------------------------------------------------------
// 3.  Auto helper — same idea: keep the original signature, let it forward
//                  to CPU  or GPU depending on the first frame type.
// ---------------------------------------------------------------------------
PointCloud::Ptr
merge_frames_auto(const std::vector<Eigen::Isometry3d>& poses,
                  const std::vector<PointCloud::ConstPtr>& frames,
                  double downsample_resolution);

}  // namespace gtsam_points
