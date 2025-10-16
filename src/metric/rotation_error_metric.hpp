#pragma once

#include "metric_base.hpp"
#include <nlohmann/json.hpp>

class RotationErrorMetric : public MetricBase {
public:
  explicit RotationErrorMetric(const nlohmann::json &config);

  double evaluate(const std::vector<TransMat> &estimated,
                  const std::vector<TransMat> &ground_truth) override;

  std::string name() const override;

  static std::shared_ptr<MetricBase> create(const nlohmann::json &config);

private:
  double compute_angle(const TransMat &estimated,
                       const TransMat &ground_truth) const;

  bool _output_in_degrees{true};
};
