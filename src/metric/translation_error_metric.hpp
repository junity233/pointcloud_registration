#pragma once

#include "metric_base.hpp"
#include <nlohmann/json.hpp>

class TranslationErrorMetric : public MetricBase {
public:
  explicit TranslationErrorMetric(const nlohmann::json &config);

  double evaluate(const std::vector<TransMat> &estimated,
                  const std::vector<TransMat> &ground_truth) override;

  std::string name() const override;

  static std::shared_ptr<MetricBase> create(const nlohmann::json &config);

private:
  bool _use_root_mean_square{false};
};
