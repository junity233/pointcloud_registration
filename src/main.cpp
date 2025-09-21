#include <atomic>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <BS_thread_pool.hpp>
#include <csv.hpp>
#include <print>
#include <ranges>

#include "algorithm/algorithm_base.hpp"
#include "dataset_loader/dataset_loader_base.hpp"
#include "metric/metric_base.hpp"
int main(int argc, char **argv) {
    nlohmann::json config;
    std::ifstream config_file("config.json");

    std::vector<std::shared_ptr<AlgorithmBase>> algorithms;
    std::shared_ptr<DatasetLoaderBase> dataset_loader;
    std::vector<std::shared_ptr<MetricBase>> metrics;

    if (!config_file.is_open()) {
        std::cout << "Error: Could not open config.json" << std::endl;
        return -1;
    }

    try {
        config_file >> config;
    } catch (const std::exception &e) {
        std::cout << "Error: Failed to parse config.json: " << e.what() << std::endl;
        return -1;
    }

    if (!config.contains("algorithms") || !config["algorithms"].is_array()) {
        std::cout << "Error: config.algorithms must be an array" << std::endl;
        return -1;
    }
    if (!config.contains("metrics") || !config["metrics"].is_array()) {
        std::cout << "Error: config.metrics must be an array" << std::endl;
        return -1;
    }
    if (!config.contains("dataset_loader") || !config["dataset_loader"].is_object()) {
        std::cout << "Error: config.dataset_loader must be an object" << std::endl;
        return -1;
    }
    if (!config["dataset_loader"].contains("name") || !config["dataset_loader"]["name"].is_string()) {
        std::cout << "Error: config.dataset_loader.name must be a string" << std::endl;
        return -1;
    }

    for (const auto &algorithm_config : config["algorithms"]) {
        if (!algorithm_config.contains("name") || !algorithm_config["name"].is_string()) {
            std::cout << "Error: Each algorithm config must have a string 'name'" << std::endl;
            return -1;
        }
        try {
            auto algorithm =
                algorithmManager.create(algorithm_config["name"], algorithm_config);
            algorithms.emplace_back(std::move(algorithm));
        } catch (const std::exception &e) {
            std::cout << "Error creating algorithm '" << algorithm_config["name"].get<std::string>()
                      << "': " << e.what() << std::endl;
            return -1;
        }
    }

    for (const auto &metric_config : config["metrics"]) {
        if (!metric_config.contains("name") || !metric_config["name"].is_string()) {
            std::cout << "Error: Each metric config must have a string 'name'" << std::endl;
            return -1;
        }
        try {
            auto metric = metricManager.create(metric_config["name"], metric_config);
            metrics.emplace_back(std::move(metric));
        } catch (const std::exception &e) {
            std::cout << "Error creating metric '" << metric_config["name"].get<std::string>()
                      << "': " << e.what() << std::endl;
            return -1;
        }
    }

    try {
        dataset_loader = datasetLoaderManager.create(
            config["dataset_loader"]["name"], config["dataset_loader"]);
    } catch (const std::exception &e) {
        std::cout << "Error creating dataset loader '" << config["dataset_loader"]["name"].get<std::string>()
                  << "': " << e.what() << std::endl;
        return -1;
    }

    auto point_clouds = dataset_loader->load_point_clouds();

    auto thread_pool = BS::thread_pool{std::thread::hardware_concurrency() + 1};

    std::map<std::string, std::vector<std::future<std::vector<double>>>>
        result_futures;

    const int task_total = static_cast<int>(algorithms.size() * point_clouds.size());
    std::atomic_int completed_tasks = 0;

    for (auto &algorithm : algorithms) {
        for (const auto &point_cloud : point_clouds) {
            auto algo = algorithm; // copy shared_ptr (stable in lambda)
            auto pc_ptr = &point_cloud; // stable pointer to element
            auto future = thread_pool.submit_task([algo, pc_ptr, &metrics, &completed_tasks, task_total]() {
                auto result = algo->register_point_cloud(*pc_ptr);
                std::vector<double> metric_results;
                metric_results.reserve(metrics.size());

                for (const auto &metric : metrics) {
                    auto metric_result = metric->evaluate(result, *pc_ptr);
                    metric_results.emplace_back(metric_result);
                }

                completed_tasks.store(completed_tasks.load() + 1);
                std::print("Complete {}/{}\r", completed_tasks.load(), task_total);

                return metric_results;
            });

            result_futures[algorithm->name()].emplace_back(std::move(future));
        }
    }

    thread_pool.wait();

    std::map<std::string, std::vector<std::vector<double>>> results;

    for (auto& [algorithm_name, futures] : result_futures)
    {
        auto &algorithm_result = results[algorithm_name];
        for (auto& future : futures)
             algorithm_result.emplace_back(future.get());

        std::ofstream csv_file(algorithm_name + "_result.csv");
        auto writer = csv::make_csv_writer(csv_file);

        writer << (metrics | std::views::transform([](std::shared_ptr<MetricBase> metric) {
            return metric->name();
        }));

        for (const auto& result : algorithm_result)
            writer << result;
    }

    return 0;
}
