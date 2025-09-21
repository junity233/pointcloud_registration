#pragma once
#include "common.hpp"
#include "singleton.hpp"
#include <nlohmann/json.hpp>
#include <memory>
#include <map>
#include <functional>
#include <stdexcept>

class DatasetLoaderBase {
public:
  virtual ~DatasetLoaderBase() = default;
  virtual std::vector<PointCloud> load_point_clouds() = 0;
};

class DatasetLoaderManager : public Singleton<DatasetLoaderManager> {
public:
  using DatasetLoaderCreateFunc = std::function<std::shared_ptr<DatasetLoaderBase>(const nlohmann::json &)>;
  virtual ~DatasetLoaderManager() = default;

  inline std::shared_ptr<DatasetLoaderBase> create(const std::string& name,const nlohmann::json &config) {
    auto it = _datasetLoaders.find(name);
    if (it == _datasetLoaders.end()) {
      throw std::runtime_error("Dataset loader not registered: " + name);
    }
    return it->second(config);
  }

  inline void register_datasetLoader(const std::string& name,DatasetLoaderCreateFunc func) {
    _datasetLoaders[name] = func;
  }

private:
  std::map<std::string,DatasetLoaderCreateFunc> _datasetLoaders;
};

#define datasetLoaderManager (DatasetLoaderManager::instance())

template <typename T>
struct DatasetLoaderRegistrar {
    DatasetLoaderRegistrar(const std::string& name) {
        datasetLoaderManager.register_datasetLoader(name, &T::create);
    }
};

#define REGISTER_DATASET_LOADER(name, class_name) \
  static DatasetLoaderRegistrar<class_name> reg_##name(#name)
