// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "google/protobuf/descriptor.h"
#include "google/protobuf/descriptor.pb.h"
#include "yaml-cpp/yaml.h"

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/executor/timer.h"
#include "aimrt_module_cpp_interface/util/buffer.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "core/util/topic_meta_key.h"
#include "mcap/writer.hpp"
#include "record_playback.aimrt_rpc.pb.h"
#include "record_playback_plugin/metadata_yaml.h"
#include "record_playback_plugin/topic_meta.h"
#include "ros_storage.h"
#include "topic_meta.h"

namespace aimrt::plugins::record_playback_plugin {

class RecordAction {
 public:
  struct Options {
    std::string bag_path;
    enum class Mode {
      kImd,
      kSignal,
    };
    Mode mode = Mode::kImd;

    struct StoragePolicy {
      uint32_t max_bag_size_m = 2048;
      uint32_t max_bag_num = 0;
      uint32_t msg_write_interval = 1000;
      uint32_t msg_write_interval_time = 1000;
      std::string compression_mode = "zstd";
      std::string compression_level = "default";
    };

    StoragePolicy storage_policy;

    YAML::Node extra_attributes;

    uint64_t max_preparation_duration_s = 0;
    std::string executor;

    struct TopicMeta {
      std::string topic_name;
      std::string msg_type;
      std::string serialization_type;
    };
    std::vector<TopicMeta> topic_meta_list;
  };

  struct OneRecord {
    uint64_t timestamp;
    uint64_t topic_index;
    std::shared_ptr<aimrt::util::BufferArrayView> buffer_view_ptr;
  };

 public:
  RecordAction() = default;
  ~RecordAction() = default;

  RecordAction(const RecordAction&) = delete;
  RecordAction& operator=(const RecordAction&) = delete;

  void Initialize(YAML::Node options_node);
  void Start();
  void Shutdown();

  void InitExecutor(aimrt::executor::ExecutorRef);

  const Options& GetOptions() const { return options_; }

  void RegisterGetExecutorFunc(
      const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func);

  void RegisterGetTypeSupportFunc(
      const std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func);

  const auto& GetTopicMetaMap() const { return topic_meta_map_; }
  void AddRecord(OneRecord&& record);

  bool StartSignalRecord(uint64_t preparation_duration_s, uint64_t record_duration_s);
  void StopSignalRecord();

  void UpdateMetadata(
      const google::protobuf::Map<std::string, std::string>& kv_pairs);

 private:
  void AddRecordImpl(OneRecord&& record);
  void OpenNewMcapToRecord(uint64_t start_timestamp);
  void CloseRecord();
  void FlushToDisk();

  void SetMcapOptions();

  size_t GetFileSize() const;

  std::string BuildROS2Schema(const MessageMembers* members, int indent);
  google::protobuf::FileDescriptorSet BuildPbSchema(const google::protobuf::Descriptor* toplevelDescriptor);

  size_t GetDbFileSize() const;

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;

  std::function<executor::ExecutorRef(std::string_view)> get_executor_func_;
  aimrt::executor::ExecutorRef executor_;

  struct McapStruct {
    std::string schema_name;
    std::string schema_format;
    std::string schema_data;
    std::string channel_name;
    std::string channel_format;
  };

  struct {
    mcap::Compression compression_mode;
    mcap::CompressionLevel compression_level;
  } mcap_options;

  std::string bag_base_name_;
  std::filesystem::path real_bag_path_;

  std::unordered_map<uint64_t, McapStruct> mcap_info_map_;  // use to record
  std::unordered_map<uint64_t, uint16_t> topic_id_to_channel_id_map_;

  std::string file_path_;
  std::string cur_mcap_file_path_;
  uint32_t cur_mcap_file_index_ = 0;
  std::unique_ptr<mcap::McapWriter> writer_;
  size_t cur_data_size_;
  double estimated_overhead_ = 1.5;

  std::function<aimrt::util::TypeSupportRef(std::string_view)> get_type_support_func_;
  std::unordered_map<aimrt::runtime::core::util::TopicMetaKey, TopicMeta,
                     aimrt::runtime::core::util::TopicMetaKey::Hash>
      topic_meta_map_;

  std::shared_ptr<aimrt::executor::TimerBase> sync_timer_;

  size_t max_bag_size_ = 0;

  size_t cur_exec_count_ = 0;

  std::filesystem::path metadata_yaml_file_path_;
  std::mutex metadata_mutex_;
  MetaData metadata_;

  // only for signal mode
  bool recording_flag_ = false;
  uint64_t max_preparation_duration_ns_ = 0;
  uint64_t stop_record_timestamp_ = 0;

  std::deque<OneRecord> last_cache_;
  std::deque<OneRecord> cur_cache_;
};

}  // namespace aimrt::plugins::record_playback_plugin
