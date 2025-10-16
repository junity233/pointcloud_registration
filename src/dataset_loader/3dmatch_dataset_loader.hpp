#pragma once

#include "dataset_loader_base.hpp"
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

class DatasetLoader3DMatch : public DatasetLoaderBase {
public:
  explicit DatasetLoader3DMatch(const nlohmann::json &config);

  std::vector<Sample> load_samples() override;

  static std::shared_ptr<DatasetLoaderBase>
  create(const nlohmann::json &config);

  std::string name() const override { return "3dmatch"; }

private:
  Sample load_sequence(const std::filesystem::path &sequence_path) const;
  PointCloud load_point_cloud(const std::filesystem::path &path) const;
  TransMat load_pose(const std::filesystem::path &path) const;

  std::filesystem::path _root;
  std::string _split;
  std::vector<std::string> _sequences;
  std::size_t _max_sequences{0};
  std::size_t _max_point_clouds{0};
};
