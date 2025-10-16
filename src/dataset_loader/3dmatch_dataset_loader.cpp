#include "dataset_loader/3dmatch_dataset_loader.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <pcl/io/ply_io.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "dataset_loader_base.hpp"

using namespace std::string_literals;

namespace fs = std::filesystem;

namespace {
bool register_loader() {
  datasetLoaderManager.register_datasetLoader(
      "3dmatch", &DatasetLoader3DMatch::create);
  return true;
}

const bool registered = register_loader();
} // namespace

DatasetLoader3DMatch::DatasetLoader3DMatch(const nlohmann::json &config) {
  if (config.contains("root")) {
    if (!config["root"].is_string()) {
      throw std::invalid_argument(
          "DatasetLoader3DMatch expects 'root' to be a string when provided");
    }
    _root = fs::path(config["root"].get<std::string>());
  } else {
    _root = fs::path("datasets/3dmatch");
  }
  _split = config.value("split", "train");
  _max_sequences = config.value("max_sequences", static_cast<std::size_t>(0));
  _max_point_clouds =
      config.value("max_point_clouds", static_cast<std::size_t>(0));

  if (config.contains("sequences")) {
    if (!config["sequences"].is_array()) {
      throw std::invalid_argument("'sequences' must be an array of strings");
    }
    for (const auto &seq : config["sequences"]) {
      if (!seq.is_string()) {
        throw std::invalid_argument(
            "'sequences' must contain only string elements");
      }
      _sequences.emplace_back(seq.get<std::string>());
    }
  }

  if (!fs::exists(_root)) {
    throw std::runtime_error("3DMatch root directory does not exist: " +
                             _root.string());
  }
}

std::vector<Sample> DatasetLoader3DMatch::load_samples() {
  std::vector<Sample> samples;
  const fs::path split_path = _root / _split;
  if (!fs::exists(split_path)) {
    throw std::runtime_error("3DMatch split directory does not exist: " +
                             split_path.string());
  }

  log_info("Loading 3DMatch dataset from {}", split_path.string());

  std::vector<fs::path> sequence_paths;
  if (!_sequences.empty()) {
    for (const auto &sequence_name : _sequences) {
      const auto sequence_path = split_path / sequence_name;
      if (fs::exists(sequence_path) && fs::is_directory(sequence_path)) {
        sequence_paths.emplace_back(sequence_path);
      } else {
        log_warn("Sequence directory missing: {}",
                  sequence_path.string());
      }
    }
  } else {
    for (const auto &entry : fs::directory_iterator(split_path)) {
      if (entry.is_directory()) {
        sequence_paths.emplace_back(entry.path());
      }
    }
    std::sort(sequence_paths.begin(), sequence_paths.end());
  }

  std::size_t sequence_count = 0;
  std::size_t cloud_count = 0;
  for (const auto &sequence_path : sequence_paths) {
    if (_max_sequences > 0 && sequence_count >= _max_sequences) {
      break;
    }
    log_info("Loading sequence {}", sequence_path.filename().string());

    try {
      auto sample = load_sequence(sequence_path);
      if (!sample.point_clouds.empty()) {
        cloud_count += sample.point_clouds.size();
        samples.emplace_back(std::move(sample));
        ++sequence_count;
      }
    } catch (const std::exception &e) {
      log_warn("Skipping sequence '{}' due to error: {}",
                sequence_path.string(), e.what());
    }
  }

  log_info("Loaded {} samples", samples.size());
  return samples;
}

Sample DatasetLoader3DMatch::load_sequence(const fs::path &sequence_path) const {
  const fs::path fragments_dir = sequence_path / "fragments";
  const fs::path poses_dir = sequence_path / "poses";

  if (!fs::exists(fragments_dir) || !fs::is_directory(fragments_dir)) {
    throw std::runtime_error("Missing fragments directory: " +
                             fragments_dir.string());
  }
  if (!fs::exists(poses_dir) || !fs::is_directory(poses_dir)) {
    throw std::runtime_error("Missing poses directory: " +
                             poses_dir.string());
  }

  std::map<std::size_t, fs::path> indexed_clouds;
  for (const auto &entry : fs::directory_iterator(fragments_dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    const auto stem = entry.path().stem().string();
    constexpr std::string_view prefix = "cloud_bin_";
    if (!stem.starts_with(prefix)) {
      continue;
    }

    try {
      const auto index = static_cast<std::size_t>(
          std::stoul(stem.substr(prefix.size())));
      indexed_clouds.emplace(index, entry.path());
    } catch (const std::exception &) {
      continue;
    }
  }

  if (indexed_clouds.empty()) {
    throw std::runtime_error("No cloud_bin_*.ply files found in " +
                             fragments_dir.string());
  }

  Sample sample;

  std::size_t loaded = 0;
  for (const auto &[index, cloud_path] : indexed_clouds) {
    if (_max_point_clouds > 0 && loaded >= _max_point_clouds) {
      break;
    }

    const fs::path pose_path =
        poses_dir / ("cloud_bin_" + std::to_string(index) + ".txt");

    if (!fs::exists(pose_path)) {
      log_warn("Skipping cloud {} due to missing pose file", cloud_path.string());
      continue;
    }

    PointCloud cloud = load_point_cloud(cloud_path);
    TransMat pose = load_pose(pose_path);

    sample.point_clouds.emplace_back(std::move(cloud));
    sample.world_transforms.emplace_back(std::move(pose));
    ++loaded;
  }

  if (sample.point_clouds.size() != sample.world_transforms.size()) {
    throw std::runtime_error(
        "Loaded point clouds and poses count mismatch in sequence " +
        sequence_path.string());
  }

  log_info("Sequence '{}' loaded with {} point clouds",
           sequence_path.filename().string(), sample.point_clouds.size());

  return sample;
}

PointCloud
DatasetLoader3DMatch::load_point_cloud(const fs::path &path) const {
  PointCloud cloud;
  const auto ret = pcl::io::loadPLYFile(path.string(), cloud);
  if (ret != 0) {
    throw std::runtime_error("Failed to load PLY file: " + path.string() +
                             " (error code " + std::to_string(ret) + ")");
  }
  return cloud;
}

TransMat DatasetLoader3DMatch::load_pose(const fs::path &path) const {
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("Failed to open pose file: " + path.string());
  }

  std::string header_line;
  if (!std::getline(in, header_line)) {
    throw std::runtime_error("Pose file is empty: " + path.string());
  }

  TransMat pose = TransMat::Identity();
  for (int row = 0; row < 4; ++row) {
    std::string line;
    if (!std::getline(in, line)) {
      throw std::runtime_error("Pose file has incomplete matrix: " +
                               path.string());
    }
    std::istringstream iss(line);
    for (int col = 0; col < 4; ++col) {
      if (!(iss >> pose(row, col))) {
        throw std::runtime_error("Failed to parse pose value at row " +
                                 std::to_string(row) + ", col " +
                                 std::to_string(col) + " in file " +
                                 path.string());
      }
    }
  }

  return pose;
}

std::shared_ptr<DatasetLoaderBase>
DatasetLoader3DMatch::create(const nlohmann::json &config) {
  return std::make_shared<DatasetLoader3DMatch>(config);
}
