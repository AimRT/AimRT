// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/util/buffer.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "core/util/topic_meta_key.h"
#include "record_playback_plugin/metadata_yaml.h"
#include "record_playback_plugin/storage/mcap_storage.h"
#include "record_playback_plugin/storage/sqlite_storage.h"
#include "record_playback_plugin/topic_meta.h"

#include "sqlite3.h"
#include "yaml-cpp/yaml.h"

namespace aimrt::plugins::record_playback_plugin {

class PlaybackAction {
 public:
  struct Options {
    std::string bag_path;

    enum class Mode {
      kImd,
      kSignal,
    };
    Mode mode = Mode::kImd;

    std::string executor;

    uint64_t skip_duration_s = 0;
    uint64_t play_duration_s = 0;

    struct TopicMeta {
      std::string topic_name;
      std::string msg_type;
    };
    std::vector<TopicMeta> topic_meta_list;
  };

  struct OneRecord {
    uint64_t topic_index;
    uint64_t dt;
    std::shared_ptr<aimrt::util::BufferArrayView> buffer_view_ptr;
  };

 public:
  PlaybackAction() = default;
  ~PlaybackAction() = default;

  PlaybackAction(const PlaybackAction&) = delete;
  PlaybackAction& operator=(const PlaybackAction&) = delete;

  void Initialize(YAML::Node options_node);
  void Start();
  void Shutdown();

  void InitExecutor();

  const Options& GetOptions() const { return options_; }

  void RegisterGetExecutorFunc(
      const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func);

  void RegisterGetTypeSupportFunc(
      const std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func);

  const auto& GetTopicMetaMap() const { return topic_meta_map_; }
  void RegisterPubRecordFunc(std::function<void(const OneRecord&)>&& func);

  bool StartSignalPlayback(uint64_t skip_duration_s, uint64_t play_duration_s);
  void StopSignalPlayback();

 private:
  void StartPlaybackImpl(uint64_t skip_duration_s, uint64_t play_duration_s);

  void AddPlaybackTasks(const std::shared_ptr<void>& task_counter_ptr);

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;

  std::unique_ptr<StorageInterface> storage_;

  std::function<executor::ExecutorRef(std::string_view)> get_executor_func_;
  aimrt::executor::ExecutorRef executor_;

  std::function<aimrt::util::TypeSupportRef(std::string_view)> get_type_support_func_;
  std::unordered_map<uint64_t, TopicMeta> topic_meta_map_;

  MetaData metadata_;

  std::function<void(const OneRecord&)> pub_record_func_;

  uint64_t start_timestamp_;

  uint32_t cur_storage_file_index_ = 0;
  uint64_t start_playback_timestamp_ = 0;
  uint64_t stop_playback_timestamp_ = 0;

  // sqlite3* db_ = nullptr;
  // sqlite3_stmt* select_msg_stmt_ = nullptr;
  std::string select_msg_sql_topic_id_range_;
  std::mutex db_mutex_;

  enum class PlayBackState {
    kReadyToPlay,
    kPlaying,
    kGetStopSignal,
  };
  PlayBackState playback_state_ = PlayBackState::kReadyToPlay;
  std::mutex playback_state_mutex_;
};

}  // namespace aimrt::plugins::record_playback_plugin