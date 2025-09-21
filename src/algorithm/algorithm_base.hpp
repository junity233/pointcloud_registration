#pragma once
#include "common.hpp"
#include "singleton.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <memory>
#include <map>
#include <functional>
#include <stdexcept>


class AlgorithmBase {
public:
  virtual ~AlgorithmBase() = default;
  virtual std::string name() const = 0;
  virtual PointCloud register_point_cloud(const PointCloud& source) = 0;
};

class AlgorithmManager : public Singleton<AlgorithmManager> {
public:
  using AlgorithmCreateFunc = std::function<std::shared_ptr<AlgorithmBase>(const nlohmann::json &)>;
  virtual ~AlgorithmManager() = default;

  inline std::shared_ptr<AlgorithmBase> create(const std::string& name,const nlohmann::json &config) {
    auto it = _algorithms.find(name);
    if (it == _algorithms.end()) {
      throw std::runtime_error("Algorithm not registered: " + name);
    }
    return it->second(config);
  }

  inline void register_algorithm(const std::string& name,AlgorithmCreateFunc func) {
    _algorithms[name] = func;
  }

private:
  std::map<std::string,AlgorithmCreateFunc> _algorithms;
};

#define algorithmManager (AlgorithmManager::instance())

template <typename T>
struct AlgorithmRegistrar {
    AlgorithmRegistrar(const std::string& name) {
        algorithmManager.register_algorithm(name, &T::create);
    }
};

#define REGISTER_ALGORITHM(name, class_name) \
  static AlgorithmRegistrar<class_name> reg_##name(#name)
