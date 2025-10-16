#pragma once
#include "common.hpp"
#include "singleton.hpp"
#include <nlohmann/json.hpp>
#include <memory>
#include <map>
#include <functional>
#include <stdexcept>
#include <vector>

class MetricBase {
public:
  virtual ~MetricBase() = default;
  virtual double evaluate(const std::vector<TransMat>& estimated,
                          const std::vector<TransMat>& ground_truth) = 0;
  virtual std::string name() const = 0;
};

using Metric = MetricBase*;

class MetricManager : public Singleton<MetricManager> {
public:
  using MetricCreateFunc = std::function<std::shared_ptr<MetricBase>(const nlohmann::json &)>;
  virtual ~MetricManager() = default;

  inline std::shared_ptr<MetricBase> create(const std::string& name,const nlohmann::json &config) {
    auto it = _metrics.find(name);
    if (it == _metrics.end()) {
      throw std::runtime_error("Metric not registered: " + name);
    }
    return it->second(config);
  }

  inline void register_metric(const std::string& name,MetricCreateFunc func) {
    _metrics[name] = func;
  }

private:
  std::map<std::string,MetricCreateFunc> _metrics;
};

#define metricManager (MetricManager::instance())

template <typename T>
struct MetricRegistrar {
    MetricRegistrar(const std::string& name) {
        metricManager.register_metric(name, &T::create);
    }
};

#define REGISTER_METRIC(name, class_name) \
  static MetricRegistrar<class_name> reg_##name(#name)
