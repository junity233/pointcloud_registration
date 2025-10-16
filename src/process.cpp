#include "process.h"
#include "algorithm/algorithm_base.hpp"
#include "common.hpp"
#include "logger.hpp"
#include <BS_thread_pool.hpp>
#include <atomic>
#include <fstream>
#include <future>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

std::vector<TransMat> register_sample(AlgorithmBase &algorithm,
                                      const std::vector<PointCloud> &point_clouds) {
    std::vector<TransMat> transforms;
    transforms.reserve(point_clouds.size());

    if (point_clouds.empty()) {
        return transforms;
    }

    transforms.emplace_back(TransMat::Identity());

    for (size_t idx = 1; idx < point_clouds.size(); ++idx) {
        const auto &source = point_clouds[idx];
        const auto &target = point_clouds[idx - 1];
        const auto relative = algorithm.register_point_cloud(source, target);
        transforms.emplace_back(transforms.back() * relative);
    }

    return transforms;
}

std::vector<double> evaluate_sample(
    const std::vector<std::shared_ptr<MetricBase>> &metrics,
    const std::vector<TransMat> &estimated_transforms,
    const std::vector<TransMat> &ground_truth_transforms) {
    if (estimated_transforms.size() != ground_truth_transforms.size()) {
        throw std::invalid_argument(
            "evaluate_sample requires estimated and ground truth transforms to have equal length");
    }

    std::vector<double> scores;
    scores.reserve(metrics.size());

    for (const auto &metric : metrics) {
        scores.emplace_back(
            metric->evaluate(estimated_transforms, ground_truth_transforms));
    }

    return scores;
}

namespace {
constexpr std::string_view ROLE_PROCESS{"process"};
} // namespace

AlgorithmResults run_evaluation(
    const std::vector<std::shared_ptr<AlgorithmBase>> &algorithms,
    const std::vector<Sample> &samples,
    const std::vector<std::shared_ptr<MetricBase>> &metrics,
    std::size_t thread_count_hint) {
    AlgorithmResults results;

    if (algorithms.empty() || samples.empty()) {
        return results;
    }

    unsigned int default_threads = std::thread::hardware_concurrency();
    if (default_threads == 0) {
        default_threads = 1;
    }

    const unsigned int thread_count =
        thread_count_hint > 0 ? static_cast<unsigned int>(thread_count_hint)
                              : default_threads;
    BS::thread_pool thread_pool(thread_count);

    LOG_INFO(ROLE_PROCESS, "Starting evaluation with {} thread(s), {} algorithm(s) and {} sample(s)",
             thread_count, algorithms.size(), samples.size());

    const std::size_t total_tasks = algorithms.size() * samples.size();
    std::atomic<std::size_t> completed_tasks{0};

    for (const auto &algorithm : algorithms) {
        const auto algorithm_name = algorithm->name();
        auto &sample_scores = results[algorithm_name];
        sample_scores.resize(samples.size());

        LOG_INFO(ROLE_PROCESS, "Evaluating algorithm '{}' on {} samples",
                 algorithm_name, samples.size());

        std::vector<std::future<std::vector<double>>> futures;
        futures.reserve(samples.size());

        for (std::size_t sample_idx = 0; sample_idx < samples.size(); ++sample_idx) {
            const Sample *sample_ptr = &samples[sample_idx];
            auto algo_ptr = algorithm;

            futures.emplace_back(thread_pool.submit_task(
                [algo_ptr, sample_ptr, &metrics, &completed_tasks, total_tasks]() {
                    auto update_progress = [&completed_tasks, total_tasks]() {
                        const auto finished = completed_tasks.fetch_add(1) + 1;
                        const double ratio = (total_tasks == 0)
                                                 ? 1.0
                                                 : static_cast<double>(finished) /
                                                       static_cast<double>(total_tasks);
                        Logger::instance().progress(ratio, finished, total_tasks);
                    };

                    try {
                        const auto &point_clouds = sample_ptr->point_clouds;
                        const auto &ground_truth = sample_ptr->world_transforms;

                        const auto estimated_transforms =
                            register_sample(*algo_ptr, point_clouds);
                        auto scores =
                            evaluate_sample(metrics, estimated_transforms, ground_truth);

                        update_progress();
                        return scores;
                    } catch (...) {
                        update_progress();
                        throw;
                    }
                }));
        }

        for (std::size_t sample_idx = 0; sample_idx < futures.size(); ++sample_idx) {
            try {
                sample_scores[sample_idx] = futures[sample_idx].get();
            } catch (const std::exception &e) {
                LOG_ERROR(ROLE_PROCESS, "Error processing sample index {} with algorithm '{}': {}",
                          sample_idx, algorithm_name, e.what());
                sample_scores[sample_idx] = {};
            }
        }
    }

    return results;
}

void write_results_to_csv(const AlgorithmResults &results,
                          const std::vector<std::shared_ptr<MetricBase>> &metrics) {
    std::vector<std::string> metric_names;
    metric_names.reserve(metrics.size());
    for (const auto &metric : metrics) {
        metric_names.emplace_back(metric->name());
    }

    for (const auto &[algorithm_name, sample_scores] : results) {
        const auto output_path = algorithm_name + "_result.csv";
        LOG_INFO(ROLE_PROCESS, "Writing results for algorithm '{}' to {}", algorithm_name,
                 output_path);
        std::ofstream csv_file(output_path);
        if (!csv_file.is_open()) {
            LOG_ERROR(ROLE_PROCESS, "Failed to open result file for algorithm '{}'",
                      algorithm_name);
            continue;
        }

        const auto write_row = [&csv_file](const auto &row) {
            bool first = true;
            for (const auto &value : row) {
                if (!first) {
                    csv_file << ',';
                }
                first = false;
                csv_file << value;
            }
            csv_file << '\n';
        };

        write_row(metric_names);
        for (const auto &scores : sample_scores) {
            write_row(scores);
        }
    }
}
