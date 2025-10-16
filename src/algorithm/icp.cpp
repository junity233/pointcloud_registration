#include "algorithm/icp.hpp"
#include "pcl/registration/icp.h"
#include <stdexcept>
#include <string_view>
#include "logger.hpp"

REGISTER_ALGORITHM(icp, ICP);
ICP::ICP(const nlohmann::json &config) {
}

std::string ICP::name() const {
    return "icp";
}

TransMat ICP::register_point_cloud(const PointCloud &source, const PointCloud &target) {
    if (source.empty() || target.empty()) {
        throw std::runtime_error("ICP::register_point_cloud requires non-empty point clouds");
    }

    log_info("Aligning source ({} points) to target ({} points)",
        source.size(), target.size());

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source.makeShared());
    icp.setInputTarget(target.makeShared());

    PointCloud aligned;
    icp.align(aligned);

    if (!icp.hasConverged()) {
        throw std::runtime_error("ICP failed to converge on the provided point clouds");
    }

    log_info("Converged with score {}", icp.getFitnessScore());

    return icp.getFinalTransformation();
}

std::shared_ptr<AlgorithmBase> ICP::create(const nlohmann::json &config) {
    return std::make_shared<ICP>(config);
}
