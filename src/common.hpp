#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/src/Core/Matrix.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using TransMat = Eigen::Matrix4f;