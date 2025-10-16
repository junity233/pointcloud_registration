#pragma once

#include "algorithm/algorithm_base.hpp"
#include "common.hpp"
#include "dataset_loader/dataset_loader_base.hpp"
#include "metric/metric_base.hpp"
#include <memory>
#include <map>
#include <vector>

std::vector<TransMat> register_sample(AlgorithmBase &algorithm,
                                      const std::vector<PointCloud> &point_clouds);

std::vector<double> evaluate_sample(
    const std::vector<std::shared_ptr<MetricBase>> &metrics,
    const std::vector<TransMat> &estimated_transforms,
    const std::vector<TransMat> &ground_truth_transforms);

using SampleScores = std::vector<std::vector<double>>;
using AlgorithmResults = std::map<std::string, SampleScores>;

AlgorithmResults run_evaluation(
    const std::vector<std::shared_ptr<AlgorithmBase>> &algorithms,
    const std::vector<Sample> &samples,
    const std::vector<std::shared_ptr<MetricBase>> &metrics,
    std::size_t thread_count_hint);

void write_results_to_csv(const AlgorithmResults &results,
                          const std::vector<std::shared_ptr<MetricBase>> &metrics);
