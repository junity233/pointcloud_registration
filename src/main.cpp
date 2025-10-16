#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <cxxopts.hpp>
#include <nlohmann/json.hpp>

#include "algorithm/algorithm_base.hpp"
#include "dataset_loader/dataset_loader_base.hpp"
#include "metric/metric_base.hpp"
#include "pcl/console/print.h"
#include "process.h"
#include "logger.hpp"

namespace {

constexpr std::string_view ROLE_MAIN{"main"};

bool validate_config(const nlohmann::json &config) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    if (!config.contains("algorithms") || !config["algorithms"].is_array()) {
        LOG_ERROR(ROLE_MAIN, "config.algorithms must be an array");
        return false;
    }

    if (!config.contains("metrics") || !config["metrics"].is_array()) {
        LOG_ERROR(ROLE_MAIN, "config.metrics must be an array");
        return false;
    }

    if (!config.contains("dataset_loader") || !config["dataset_loader"].is_object()) {
        LOG_ERROR(ROLE_MAIN, "config.dataset_loader must be an object");
        return false;
    }

    if (!config["dataset_loader"].contains("name") ||
        !config["dataset_loader"]["name"].is_string()) {
        LOG_ERROR(ROLE_MAIN, "config.dataset_loader.name must be a string");
        return false;
    }

    return true;
}

} // namespace

std::string join_names(const std::vector<std::string> &names) {
    std::ostringstream oss;
    for (size_t idx = 0; idx < names.size(); ++idx) {
        if (idx > 0) {
            oss << ", ";
        }
        oss << names[idx];
    }
    return oss.str();
}

int main(int argc, char **argv) {
    cxxopts::Options options("Pointcloud Registration Evaluator",
                             "Evaluate point cloud registration algorithms");
    
    options.add_options()("c,config", "Path to config file",
                          cxxopts::value<std::string>()->default_value("config.json"))
                        ("h,help", "Print help");
    
    auto parsed_options = options.parse(argc, argv);
    if (parsed_options.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }
    
    auto config_path = parsed_options["config"].as<std::string>();


    nlohmann::json config;
    std::ifstream config_file(config_path);

    if (!config_file.is_open()) {
        LOG_ERROR(ROLE_MAIN, "Could not open {}", config_path);
        return -1;
    }

    LOG_INFO(ROLE_MAIN, "Loading config from {}", config_path);
    try {
        config_file >> config;
    } catch (const std::exception &e) {
        LOG_ERROR(ROLE_MAIN, "Failed to parse {}: {}", config_path, e.what());
        return -1;
    }

    if (!validate_config(config)) {
        return -1;
    }
    LOG_INFO(ROLE_MAIN, "Configuration validated");

    std::vector<std::shared_ptr<AlgorithmBase>> algorithms;
    algorithms.reserve(config["algorithms"].size());

    for (const auto &algorithm_config : config["algorithms"]) {
        if (!algorithm_config.contains("name") ||
            !algorithm_config["name"].is_string()) {
            LOG_ERROR(ROLE_MAIN, "Each algorithm config must have a string 'name'");
            return -1;
        }

        const auto algorithm_name = algorithm_config["name"].get<std::string>();
        try {
            auto algorithm =
                algorithmManager.create(algorithm_name, algorithm_config);
            algorithms.emplace_back(std::move(algorithm));
            LOG_INFO(ROLE_MAIN, "Initialized algorithm '{}'", algorithm_name);
        } catch (const std::exception &e) {
            LOG_ERROR(ROLE_MAIN, "Error creating algorithm '{}': {}", algorithm_name, e.what());
            return -1;
        }
    }

    std::vector<std::shared_ptr<MetricBase>> metrics;
    metrics.reserve(config["metrics"].size());

    for (const auto &metric_config : config["metrics"]) {
        if (!metric_config.contains("name") || !metric_config["name"].is_string()) {
            LOG_ERROR(ROLE_MAIN, "Each metric config must have a string 'name'");
            return -1;
        }

        const auto metric_name = metric_config["name"].get<std::string>();
        try {
            auto metric = metricManager.create(metric_name, metric_config);
            metrics.emplace_back(std::move(metric));
            LOG_INFO(ROLE_MAIN, "Initialized metric '{}'", metric_name);
        } catch (const std::exception &e) {
            LOG_ERROR(ROLE_MAIN, "Error creating metric '{}': {}", metric_name, e.what());
            return -1;
        }
    }

    std::shared_ptr<DatasetLoaderBase> dataset_loader;
    try {
        dataset_loader = datasetLoaderManager.create(
            config["dataset_loader"]["name"], config["dataset_loader"]);
        LOG_INFO(ROLE_MAIN, "Dataset loader '{}' ready",
                 config["dataset_loader"]["name"].get<std::string>());
    } catch (const std::exception &e) {
        LOG_ERROR(ROLE_MAIN, "Error creating dataset loader '{}': {}",
                  config["dataset_loader"]["name"].get<std::string>(), e.what());
        return -1;
    }

    std::vector<std::string> algorithm_names;
    algorithm_names.reserve(algorithms.size());
    for (const auto &algorithm : algorithms) {
        algorithm_names.emplace_back(algorithm->name());
    }

    std::vector<std::string> metric_names;
    metric_names.reserve(metrics.size());
    for (const auto &metric : metrics) {
        metric_names.emplace_back(metric->name());
    }

    const auto dataset_loader_name =
        config["dataset_loader"]["name"].get<std::string>();
    if (config["dataset_loader"].contains("split") &&
        config["dataset_loader"]["split"].is_string()) {
        LOG_INFO(ROLE_MAIN, "Dataset loader: {} (split={})", dataset_loader_name,
                 config["dataset_loader"]["split"].get<std::string>());
    } else {
        LOG_INFO(ROLE_MAIN, "Dataset loader: {}", dataset_loader_name);
    }
    LOG_INFO(ROLE_MAIN, "Algorithms: {}", join_names(algorithm_names));
    LOG_INFO(ROLE_MAIN, "Metrics: {}", join_names(metric_names));

    std::size_t thread_count_hint = 0;
    if (config.contains("runner") && config["runner"].is_object()) {
        const auto &runner_config = config["runner"];
        if (runner_config.contains("threads")) {
            const auto &threads_value = runner_config["threads"];
            if (threads_value.is_number_unsigned()) {
                thread_count_hint = threads_value.get<std::size_t>();
            } else if (threads_value.is_number_integer()) {
                const auto threads_signed = threads_value.get<long long>();
                if (threads_signed > 0) {
                    thread_count_hint = static_cast<std::size_t>(threads_signed);
                }
            }
        }
    }

    unsigned int default_threads = std::thread::hardware_concurrency();
    if (default_threads == 0) {
        default_threads = 1;
    }
    const std::size_t effective_threads =
        thread_count_hint > 0 ? thread_count_hint : default_threads;
    LOG_INFO(ROLE_MAIN, "Threads: {}", effective_threads);

    const auto samples = dataset_loader->load_samples();
    std::size_t total_point_clouds = 0;
    for (const auto &sample : samples) {
        total_point_clouds += sample.point_clouds.size();
    }
    LOG_INFO(ROLE_MAIN, "Loaded {} samples totaling {} point clouds",
             samples.size(), total_point_clouds);
    const auto results =
        run_evaluation(algorithms, samples, metrics, effective_threads);
    write_results_to_csv(results, metrics);

    LOG_INFO(ROLE_MAIN, "Evaluation completed successfully");

    return 0;
}
