// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include "aimrt_module_cpp_interface/util/buffer.h"
#include "record_playback_plugin/metadata_yaml.h"
#include "record_playback_plugin/topic_meta.h"

#include "aimrt_module_cpp_interface/util/buffer.h"

#include "aimrt_module_cpp_interface/util/type_support.h"
#include "record_playback_plugin/metadata_yaml.h"

namespace aimrt::plugins::record_playback_plugin {

class StorageInterface {
 public:
  virtual ~StorageInterface() = default;

  virtual bool InitializeRecord(const std::string& bag_path, uint64_t max_bag_size, uint64_t max_bag_num, MetaData& metadata,
                                std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func) = 0;

  virtual bool InitializePlayback(const std::string& bag_path, MetaData& metadata, uint64_t skip_duration_s, uint64_t play_duration_s) = 0;

  virtual bool WriteRecord(uint64_t topic_id, uint64_t timestamp,
                           std::shared_ptr<aimrt::util::BufferArrayView> buffer_view_ptr) = 0;

  virtual void FlushToDisk() = 0;

  virtual bool ReadRecord(uint64_t& start_playback_timestamp, uint64_t& stop_playback_timestamp,
                          uint64_t& topic_id, uint64_t& timestamp,
                          std::unique_ptr<char[]>& data, size_t& size) = 0;

  virtual void CloseRecord() = 0;

  virtual void ClosePlayback() = 0;

 public:
  std::string bag_base_name_;

  std::filesystem::path real_bag_path_;

  std::filesystem::path metadata_yaml_file_path_;

  MetaData metadata_;
  size_t max_bag_size_ = 0;
  size_t max_bag_num_ = 0;
  std::function<aimrt::util::TypeSupportRef(std::string_view)> get_type_support_func_;
};

}  // namespace aimrt::plugins::record_playback_plugin
