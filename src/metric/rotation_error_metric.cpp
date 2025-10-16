#include "metric/rotation_error_metric.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

REGISTER_METRIC(rotation_error, RotationErrorMetric);

RotationErrorMetric::RotationErrorMetric(const nlohmann::json &config) {
  _output_in_degrees = config.value("degrees", true);
}

double RotationErrorMetric::evaluate(const std::vector<TransMat> &estimated,
                                     const std::vector<TransMat> &ground_truth) {
  if (estimated.size() != ground_truth.size()) {
    throw std::invalid_argument(
        "RotationErrorMetric: estimated and ground truth transform counts must match");
  }

  if (estimated.empty()) {
    return 0.0;
  }

  double total_error = 0.0;
  for (size_t idx = 0; idx < estimated.size(); ++idx) {
    total_error += compute_angle(estimated[idx], ground_truth[idx]);
  }

  return total_error / static_cast<double>(estimated.size());
}

std::string RotationErrorMetric::name() const {
  return "rotation_error";
}

std::shared_ptr<MetricBase>
RotationErrorMetric::create(const nlohmann::json &config) {
  return std::make_shared<RotationErrorMetric>(config);
}

double RotationErrorMetric::compute_angle(const TransMat &estimated,
                                          const TransMat &ground_truth) const {
  const Eigen::Matrix3f rot_est = estimated.block<3, 3>(0, 0);
  const Eigen::Matrix3f rot_gt = ground_truth.block<3, 3>(0, 0);

  const Eigen::Matrix3f delta = rot_gt.transpose() * rot_est;

  const float cos_theta = std::clamp(
      (delta.trace() - 1.0f) * 0.5f, -1.0f, 1.0f);

  double angle = std::acos(static_cast<double>(cos_theta));
  if (_output_in_degrees) {
    angle = angle * 180.0 / std::numbers::pi;
  }

  return angle;
}
