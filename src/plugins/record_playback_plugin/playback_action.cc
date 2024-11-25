// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "record_playback_plugin/playback_action.h"
#include "record_playback_plugin/global.h"
#include "util/string_util.h"

#include <filesystem>

namespace YAML {

template <>
struct convert<aimrt::plugins::record_playback_plugin::PlaybackAction::Options> {
  using Options = aimrt::plugins::record_playback_plugin::PlaybackAction::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["bag_path"] = rhs.bag_path;

    if (rhs.mode == Options::Mode::kImd) {
      node["mode"] = "imd";
    } else if (rhs.mode == Options::Mode::kSignal) {
      node["mode"] = "signal";
    }

    node["executor"] = rhs.executor;
    node["skip_duration_s"] = rhs.skip_duration_s;
    node["play_duration_s"] = rhs.play_duration_s;

    node["topic_meta_list"] = YAML::Node();
    for (const auto& topic_meta : rhs.topic_meta_list) {
      Node topic_meta_node;
      topic_meta_node["topic_name"] = topic_meta.topic_name;
      topic_meta_node["msg_type"] = topic_meta.msg_type;
      node["topic_meta_list"].push_back(topic_meta_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    rhs.bag_path = node["bag_path"].as<std::string>();

    auto mode = aimrt::common::util::StrToLower(node["mode"].as<std::string>());
    if (mode == "imd") {
      rhs.mode = Options::Mode::kImd;
    } else if (mode == "signal") {
      rhs.mode = Options::Mode::kSignal;
    } else {
      throw aimrt::common::util::AimRTException("Invalid record mode: " + mode);
    }

    rhs.executor = node["executor"].as<std::string>();

    if (node["skip_duration_s"])
      rhs.skip_duration_s = node["skip_duration_s"].as<uint64_t>();
    if (node["play_duration_s"])
      rhs.play_duration_s = node["play_duration_s"].as<uint64_t>();

    if (node["topic_meta_list"] && node["topic_meta_list"].IsSequence()) {
      for (const auto& topic_meta_node : node["topic_meta_list"]) {
        auto topic_meta = Options::TopicMeta{
            .topic_name = topic_meta_node["topic_name"].as<std::string>(),
            .msg_type = topic_meta_node["msg_type"].as<std::string>()};

        rhs.topic_meta_list.emplace_back(std::move(topic_meta));
      }
    }

    return true;
  }
};

}  // namespace YAML

namespace aimrt::plugins::record_playback_plugin {

void PlaybackAction::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Local channel backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  AIMRT_CHECK_ERROR_THROW(
      get_type_support_func_,
      "Get type support function is not set before initialize.");

  options_.bag_path = std::filesystem::canonical(std::filesystem::absolute(options_.bag_path)).string();

  // 读取 metadata.yaml
  auto metadata_yaml_file_path = std::filesystem::path(options_.bag_path) / "metadata.yaml";

  AIMRT_CHECK_ERROR_THROW(
      std::filesystem::exists(metadata_yaml_file_path) && std::filesystem::is_regular_file(metadata_yaml_file_path),
      "Can not find 'metadata.yaml' in bag path '{}'.", options_.bag_path);

  auto metadata_root_node = YAML::LoadFile(metadata_yaml_file_path.string());
  metadata_ = metadata_root_node["aimrt_bagfile_information"].as<MetaData>();

  // 检查version
  AIMRT_CHECK_ERROR_THROW(metadata_.version == kVersion,
                          "Version inconsistency, cur plugin version: {}, bag version: {}",
                          kVersion, metadata_.version);

  // 检查 select topic meta
  if (!options_.topic_meta_list.empty()) {
    std::vector<uint32_t> enable_topic_id_vec;

    std::vector<TopicMeta> select_topics;
    for (auto item : options_.topic_meta_list) {
      auto finditr = std::find_if(
          metadata_.topics.begin(), metadata_.topics.end(),
          [&item](const auto& topic_meta) {
            return (item.topic_name == topic_meta.topic_name) && (item.msg_type == topic_meta.msg_type);
          });

      if (finditr == metadata_.topics.end()) [[unlikely]] {
        AIMRT_WARN("Can not find topic '{}' with msg type '{}' in bag '{}'.",
                   item.topic_name, item.msg_type, options_.bag_path);

        continue;
      }

      enable_topic_id_vec.emplace_back(finditr->id);

      select_topics.emplace_back(*finditr);
    }

    if (metadata_.topics.size() != select_topics.size()) {
      metadata_.topics = std::move(select_topics);

      for (size_t ii = 0; ii < enable_topic_id_vec.size(); ++ii) {
        select_msg_sql_topic_id_range_ += std::to_string(enable_topic_id_vec[ii]);
        if (ii != enable_topic_id_vec.size() - 1) {
          select_msg_sql_topic_id_range_ += ", ";
        }
      }
    }
  }

  // 检查 topic meta
  for (auto& topic_meta : metadata_.topics) {
    // 检查消息类型
    auto type_support_ref = get_type_support_func_(topic_meta.msg_type);
    AIMRT_CHECK_ERROR_THROW(type_support_ref,
                            "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);

    // 检查序列化类型
    bool check_ret = type_support_ref.CheckSerializationTypeSupported(topic_meta.serialization_type);
    AIMRT_CHECK_ERROR_THROW(check_ret,
                            "Msg type '{}' does not support serialization type '{}'.",
                            topic_meta.msg_type, topic_meta.serialization_type);

    topic_meta_map_.emplace(topic_meta.id, topic_meta);
  }

  // 检查 files
  AIMRT_CHECK_ERROR_THROW(!metadata_.files.empty(),
                          "Empty bag! bag path: {}", options_.bag_path);

  auto corrected_metadata_files = std::deque<MetaData::FileMeta>();
  for (auto& item : metadata_.files) {
    const auto db_file_path = std::filesystem::path(options_.bag_path) / item.path;
    if (!std::filesystem::exists(db_file_path) || !std::filesystem::is_regular_file(db_file_path)) {
      AIMRT_WARN("Can not find bag file '{}' in bag path '{}', this file will be ignored.", db_file_path.string(), options_.bag_path);
      continue;
    }
    corrected_metadata_files.emplace_back(std::move(item));
  }
  metadata_.files = std::move(corrected_metadata_files);

  options_node = options_;
}

void PlaybackAction::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  // start imd mode
  if (options_.mode == Options::Mode::kImd) {
    std::lock_guard<std::mutex> lck(playback_state_mutex_);
    if (playback_state_ == PlayBackState::kReadyToPlay) {
      playback_state_ = PlayBackState::kPlaying;
      playback_state_mutex_.unlock();
      StartPlaybackImpl(options_.skip_duration_s, options_.play_duration_s);
    }
  }
}

void PlaybackAction::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  std::lock_guard<std::mutex> lck(playback_state_mutex_);
  if (playback_state_ == PlayBackState::kPlaying)
    playback_state_ = PlayBackState::kGetStopSignal;
}

void PlaybackAction::InitExecutor() {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  AIMRT_CHECK_ERROR_THROW(
      get_executor_func_,
      "Get executor function is not set before initialize.");

  executor_ = get_executor_func_(options_.executor);
  AIMRT_CHECK_ERROR_THROW(
      executor_, "Can not get executor {}.", options_.executor);
  AIMRT_CHECK_ERROR_THROW(
      executor_.SupportTimerSchedule(),
      "Record executor {} do not support time schedule!", options_.executor);
}

void PlaybackAction::RegisterGetExecutorFunc(
    const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  get_executor_func_ = get_executor_func;
}

void PlaybackAction::RegisterGetTypeSupportFunc(
    const std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  get_type_support_func_ = get_type_support_func;
}

void PlaybackAction::RegisterPubRecordFunc(std::function<void(const OneRecord&)>&& func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  pub_record_func_ = std::move(func);
}

bool PlaybackAction::StartSignalPlayback(uint64_t skip_duration_s, uint64_t play_duration_s) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kStart,
      "Method can only be called when state is 'Start'.");

  if (options_.mode != Options::Mode::kSignal) [[unlikely]] {
    AIMRT_WARN("Cur action mode is not signal mode.");
    return false;
  }

  {
    std::lock_guard<std::mutex> lck(playback_state_mutex_);
    if (playback_state_ == PlayBackState::kReadyToPlay) {
      playback_state_ = PlayBackState::kPlaying;
      playback_state_mutex_.unlock();
      StartPlaybackImpl(skip_duration_s, play_duration_s);
      return true;
    }
  }

  AIMRT_WARN("Playback is already running.");
  return false;
}

void PlaybackAction::StopSignalPlayback() {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kStart,
      "Method can only be called when state is 'Start'.");

  if (options_.mode != Options::Mode::kSignal) [[unlikely]] {
    AIMRT_WARN("Cur action mode is not signal mode.");
    return;
  }

  std::lock_guard<std::mutex> lck(playback_state_mutex_);
  if (playback_state_ == PlayBackState::kPlaying)
    playback_state_ = PlayBackState::kGetStopSignal;
}

bool PlaybackAction::OpenNewDb() {
  CloseDb();

  if (cur_db_file_index_ >= metadata_.files.size()) [[unlikely]] {
    return false;
  }

  const auto& file = metadata_.files[cur_db_file_index_];
  const auto db_file_path = (std::filesystem::path(options_.bag_path) / file.path).string();
  ++cur_db_file_index_;

  uint64_t cur_db_start_timestamp = file.start_timestamp;

  if (stop_playback_timestamp_ && cur_db_start_timestamp > stop_playback_timestamp_) [[unlikely]] {
    return false;
  }

  // open db
  int ret = sqlite3_open(db_file_path.c_str(), &db_);
  AIMRT_CHECK_ERROR_THROW(ret == SQLITE_OK,
                          "Sqlite3 open db file failed, path: {}, ret: {}, error info: {}",
                          db_file_path, ret, sqlite3_errmsg(db_));

  AIMRT_TRACE("Open new db, path: {}", db_file_path);

  // create select stmt
  std::string sql = "SELECT topic_id, timestamp, data FROM messages";

  std::vector<std::string> condition;

  if (cur_db_start_timestamp < start_playback_timestamp_)
    condition.emplace_back("timestamp >= " + std::to_string(start_playback_timestamp_));

  if (!select_msg_sql_topic_id_range_.empty())
    condition.emplace_back("topic_id IN ( " + select_msg_sql_topic_id_range_ + " )");

  for (size_t ii = 0; ii < condition.size(); ++ii) {
    if (ii == 0)
      sql += " WHERE ";
    else
      sql += " AND ";

    sql += condition[ii];
  }

  AIMRT_TRACE("Sql str: {}, db path: {}", sql, db_file_path);

  ret = sqlite3_prepare_v3(db_, sql.c_str(), sql.size(), 0, &select_msg_stmt_, nullptr);
  AIMRT_CHECK_ERROR_THROW(ret == SQLITE_OK,
                          "Sqlite3 prepare failed, sql: {}, ret: {}, error info: {}",
                          sql, ret, sqlite3_errmsg(db_));

  return true;
}

void PlaybackAction::CloseDb() {
  if (db_ != nullptr) {
    if (select_msg_stmt_ != nullptr) {
      sqlite3_finalize(select_msg_stmt_);
      select_msg_stmt_ = nullptr;
    }

    sqlite3_close_v2(db_);
    db_ = nullptr;
  }
}

void PlaybackAction::StartPlaybackImpl(uint64_t skip_duration_s, uint64_t play_duration_s) {
  start_playback_timestamp_ = metadata_.files[0].start_timestamp + skip_duration_s * 1000000000;
  if (play_duration_s == 0) {
    stop_playback_timestamp_ = 0;
  } else {
    stop_playback_timestamp_ = start_playback_timestamp_ + play_duration_s * 1000000000;
  }

  size_t ii = 1;
  for (; ii < metadata_.files.size(); ++ii) {
    if (metadata_.files[ii].start_timestamp > start_playback_timestamp_)
      break;
  }
  cur_db_file_index_ = ii - 1;

  AIMRT_TRACE("Start a new playback, skip_duration_s: {}, play_duration_s: {}, start_playback_timestamp: {}, stop_playback_timestamp: {}, use db index: {}",
              skip_duration_s, play_duration_s,
              start_playback_timestamp_, stop_playback_timestamp_,
              cur_db_file_index_);

  start_timestamp_ = aimrt::common::util::GetCurTimestampNs();

  CloseDb();

  // 开始两个task包
  std::shared_ptr<void> task_counter_ptr(
      nullptr,
      [this](...) {
        std::lock_guard<std::mutex> lck(playback_state_mutex_);
        playback_state_ = PlayBackState::kReadyToPlay;
      });

  AddPlaybackTasks(task_counter_ptr);
  AddPlaybackTasks(task_counter_ptr);
}

void PlaybackAction::AddPlaybackTasks(const std::shared_ptr<void>& task_counter_ptr) {
  {
    std::lock_guard<std::mutex> lck(playback_state_mutex_);
    if (playback_state_ == PlayBackState::kGetStopSignal) [[unlikely]] {
      return;
    }
  }

  std::lock_guard<std::mutex> db_lck(db_mutex_);

  if (db_ == nullptr) [[unlikely]] {
    // first record
    if (!OpenNewDb()) {
      std::lock_guard<std::mutex> lck(playback_state_mutex_);
      playback_state_ = PlayBackState::kGetStopSignal;
      return;
    }
  }

  // 一次性吐出最多1s的数据，或最多1000条数据
  constexpr size_t kMaxRecordSize = 1000;
  uint64_t cur_start_timestamp = 0;

  std::vector<OneRecord> records;
  records.reserve(kMaxRecordSize);

  while (true) {
    int ret = sqlite3_step(select_msg_stmt_);
    if (ret == SQLITE_ROW) {
      auto topic_id = sqlite3_column_int64(select_msg_stmt_, 0);
      auto timestamp = sqlite3_column_int64(select_msg_stmt_, 1);

      if (stop_playback_timestamp_ && timestamp >= stop_playback_timestamp_) [[unlikely]] {
        std::lock_guard<std::mutex> lck(playback_state_mutex_);
        playback_state_ = PlayBackState::kGetStopSignal;
        break;
      }

      uint32_t size = sqlite3_column_bytes(select_msg_stmt_, 2);
      const void* buf = sqlite3_column_blob(select_msg_stmt_, 2);

      auto data_ptr = std::make_unique<std::string>(static_cast<const char*>(buf), size);

      aimrt_buffer_view_t buffer_view{
          .data = data_ptr->data(),
          .len = data_ptr->size()};

      aimrt_buffer_array_view_t buffer_array_view{
          .data = &buffer_view,
          .len = 1};

      records.emplace_back(
          OneRecord{
              .topic_index = static_cast<uint64_t>(topic_id),
              .dt = timestamp - start_playback_timestamp_,
              .buffer_view_ptr = std::shared_ptr<aimrt::util::BufferArrayView>(
                  new aimrt::util::BufferArrayView(buffer_array_view),
                  [data_ptr{std::move(data_ptr)}](const auto* ptr) { delete ptr; })});

      if (cur_start_timestamp == 0) [[unlikely]] {
        cur_start_timestamp = timestamp;
      } else if ((timestamp - cur_start_timestamp) >= 1000000000 || records.size() >= kMaxRecordSize) [[unlikely]] {
        break;
      }
    } else {
      if (!OpenNewDb()) {
        std::lock_guard<std::mutex> lck(playback_state_mutex_);
        playback_state_ = PlayBackState::kGetStopSignal;
        break;
      }
    }
  }

  db_mutex_.unlock();

  AIMRT_TRACE("Get {} record.", records.size());

  if (records.empty()) [[unlikely]] {
    std::lock_guard<std::mutex> lck(playback_state_mutex_);
    playback_state_ = PlayBackState::kGetStopSignal;
    return;
  }

  size_t len = records.size();
  for (size_t ii = 0; ii < len; ++ii) {
    // add publish task
    auto& record = records[ii];
    auto tp = aimrt::common::util::GetTimePointFromTimestampNs(start_timestamp_ + record.dt);

    if (ii < len - 1) {
      executor_.ExecuteAt(
          tp,
          [this, record{std::move(record)}]() {
            pub_record_func_(record);
          });
    } else {
      executor_.ExecuteAt(
          tp,
          [this, record{std::move(record)}, task_counter_ptr]() {
            pub_record_func_(record);

            // add new playback tasks to last publish task
            AddPlaybackTasks(task_counter_ptr);
          });
    }
  }
}

}  // namespace aimrt::plugins::record_playback_plugin