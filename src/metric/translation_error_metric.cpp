#include "metric/translation_error_metric.hpp"

#include <cmath>
#include <stdexcept>

REGISTER_METRIC(translation_error, TranslationErrorMetric);

TranslationErrorMetric::TranslationErrorMetric(const nlohmann::json &config) {
  _use_root_mean_square = config.value("rms", false);
}

double TranslationErrorMetric::evaluate(
    const std::vector<TransMat> &estimated,
    const std::vector<TransMat> &ground_truth) {
  if (estimated.size() != ground_truth.size()) {
    throw std::invalid_argument(
        "TranslationErrorMetric: estimated and ground truth transform counts must match");
  }

  if (estimated.empty()) {
    return 0.0;
  }

  double accumulator = 0.0;
  for (size_t idx = 0; idx < estimated.size(); ++idx) {
    const Eigen::Vector3f t_est = estimated[idx].block<3, 1>(0, 3);
    const Eigen::Vector3f t_gt = ground_truth[idx].block<3, 1>(0, 3);
    const double diff = static_cast<double>((t_est - t_gt).norm());
    if (_use_root_mean_square) {
      accumulator += diff * diff;
    } else {
      accumulator += diff;
    }
  }

  if (_use_root_mean_square) {
    return std::sqrt(accumulator / static_cast<double>(estimated.size()));
  }

  return accumulator / static_cast<double>(estimated.size());
}

std::string TranslationErrorMetric::name() const {
  return "translation_error";
}

std::shared_ptr<MetricBase>
TranslationErrorMetric::create(const nlohmann::json &config) {
  return std::make_shared<TranslationErrorMetric>(config);
}
