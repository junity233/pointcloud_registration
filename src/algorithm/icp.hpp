#pragma once
#include "algorithm_base.hpp"

class ICP : public AlgorithmBase {
public:
    explicit ICP(const nlohmann::json& config);
    std::string name() const override;
    TransMat register_point_cloud(const PointCloud& source, const PointCloud& target) override;
    static std::shared_ptr<AlgorithmBase> create(const nlohmann::json& config);
};
