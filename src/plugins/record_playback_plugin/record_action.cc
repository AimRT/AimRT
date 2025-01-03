// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "record_playback_plugin/record_action.h"

#include <fstream>
#include <future>
#include <string>

#include "log_util.h"
#include "record_playback_plugin/global.h"
#include "util/string_util.h"

namespace YAML {

template <>
struct convert<aimrt::plugins::record_playback_plugin::RecordAction::Options> {
  using Options = aimrt::plugins::record_playback_plugin::RecordAction::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["bag_path"] = rhs.bag_path;

    Node storage_policy;
    storage_policy["max_bag_size_m"] = rhs.storage_policy.max_bag_size_m;
    storage_policy["max_bag_num"] = rhs.storage_policy.max_bag_num;
    storage_policy["msg_write_interval"] = rhs.storage_policy.msg_write_interval;
    storage_policy["msg_write_interval_time"] = rhs.storage_policy.msg_write_interval_time;
    storage_policy["synchronous_mode"] = rhs.storage_policy.synchronous_mode;
    storage_policy["journal_mode"] = rhs.storage_policy.journal_mode;
    node["storage_policy"] = storage_policy;

    if (rhs.mode == Options::Mode::kImd) {
      node["mode"] = "imd";
    } else if (rhs.mode == Options::Mode::kSignal) {
      node["mode"] = "signal";
    }

    node["max_preparation_duration_s"] = rhs.max_preparation_duration_s;
    node["timer_executor"] = rhs.timer_executor;

    node["topic_meta_list"] = YAML::Node();
    for (const auto& topic_meta : rhs.topic_meta_list) {
      Node topic_meta_node;
      topic_meta_node["topic_name"] = topic_meta.topic_name;
      topic_meta_node["msg_type"] = topic_meta.msg_type;
      topic_meta_node["serialization_type"] = topic_meta.serialization_type;
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

    if (node["storage_policy"]) {
      Node storage_policy = node["storage_policy"];

      if (storage_policy["max_bag_size_m"])
        rhs.storage_policy.max_bag_size_m = storage_policy["max_bag_size_m"].as<uint32_t>();

      if (storage_policy["max_bag_num"])
        rhs.storage_policy.max_bag_num = storage_policy["max_bag_num"].as<uint32_t>();

      if (storage_policy["msg_write_interval"])
        rhs.storage_policy.msg_write_interval = storage_policy["msg_write_interval"].as<uint32_t>();

      if (storage_policy["msg_write_interval_time"])
        rhs.storage_policy.msg_write_interval_time = storage_policy["msg_write_interval_time"].as<uint32_t>();

      if (storage_policy["journal_mode"]) {
        auto journal_mode = aimrt::common::util::StrToLower(storage_policy["journal_mode"].as<std::string>());
        if (journal_mode != "delete" && journal_mode != "truncate" && journal_mode != "persist" && journal_mode != "memory" && journal_mode != "wal") {
          throw aimrt::common::util::AimRTException("Invalid journal mode: " + journal_mode);
        }
        rhs.storage_policy.journal_mode = journal_mode;
      }

      if (storage_policy["synchronous_mode"]) {
        auto synchronous_mode = aimrt::common::util::StrToLower(storage_policy["synchronous_mode"].as<std::string>());
        if (synchronous_mode != "off" && synchronous_mode != "normal" && synchronous_mode != "full" && synchronous_mode != "extra") {
          throw aimrt::common::util::AimRTException("Invalid synchronous mode: " + synchronous_mode);
        }
        rhs.storage_policy.synchronous_mode = synchronous_mode;
      }
    }

    if (node["max_preparation_duration_s"])
      rhs.max_preparation_duration_s = node["max_preparation_duration_s"].as<uint64_t>();

    rhs.timer_executor = node["timer_executor"].as<std::string>();

    if (node["topic_meta_list"] && node["topic_meta_list"].IsSequence()) {
      for (const auto& topic_meta_node : node["topic_meta_list"]) {
        auto topic_meta = Options::TopicMeta{
            .topic_name = topic_meta_node["topic_name"].as<std::string>(),
            .msg_type = topic_meta_node["msg_type"].as<std::string>()};

        if (topic_meta_node["serialization_type"])
          topic_meta.serialization_type = topic_meta_node["serialization_type"].as<std::string>();

        rhs.topic_meta_list.emplace_back(std::move(topic_meta));
      }
    }

    return true;
  }
};

}  // namespace YAML

namespace aimrt::plugins::record_playback_plugin {

void RecordAction::Initialize(YAML::Node options) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Local channel backend can only be initialized once.");

  if (options && !options.IsNull())
    options_ = options.as<Options>();

  AIMRT_CHECK_ERROR_THROW(
      get_type_support_func_,
      "Get type support function is not set before initialize.");

  // 检查 topic meta
  for (auto& topic_meta : options_.topic_meta_list) {
    // 检查消息类型
    auto type_support_ref = get_type_support_func_(topic_meta.msg_type);
    AIMRT_CHECK_ERROR_THROW(type_support_ref,
                            "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);

    // 检查序列化类型
    if (!topic_meta.serialization_type.empty()) {
      bool check_ret = type_support_ref.CheckSerializationTypeSupported(topic_meta.serialization_type);
      AIMRT_CHECK_ERROR_THROW(check_ret,
                              "Msg type '{}' does not support serialization type '{}'.",
                              topic_meta.msg_type, topic_meta.serialization_type);
    } else {
      topic_meta.serialization_type = type_support_ref.DefaultSerializationType();
    }
  }

  // bag_path
  auto tm = aimrt::common::util::GetCurTm();
  char buf[17];  // _YYYYMMDD_hhmmss
  snprintf(buf, sizeof(buf), "_%04d%02d%02d_%02d%02d%02d",
           (tm.tm_year + 1900) % 10000u, (tm.tm_mon + 1) % 100u, (tm.tm_mday) % 100u,
           (tm.tm_hour) % 100u, (tm.tm_min) % 100u, (tm.tm_sec) % 100u);
  bag_base_name_ = "aimrtbag" + std::string(buf);

  std::filesystem::path parent_bag_path = std::filesystem::absolute(options_.bag_path);
  if (!(std::filesystem::exists(parent_bag_path) && std::filesystem::is_directory(parent_bag_path))) {
    std::filesystem::create_directories(parent_bag_path);
  }
  options_.bag_path = std::filesystem::canonical(parent_bag_path).string();

  real_bag_path_ = parent_bag_path / bag_base_name_;
  AIMRT_CHECK_ERROR_THROW(!std::filesystem::exists(real_bag_path_),
                          "Bag path '{}' is exist!", real_bag_path_.string());

  std::filesystem::create_directories(real_bag_path_);

  // 初始化 metadata.yaml
  metadata_yaml_file_path_ = real_bag_path_ / "metadata.yaml";
  metadata_.version = kVersion;

  for (auto& topic_meta_option : options_.topic_meta_list) {
    runtime::core::util::TopicMetaKey key{
        .topic_name = topic_meta_option.topic_name,
        .msg_type = topic_meta_option.msg_type};

    AIMRT_CHECK_ERROR_THROW(
        topic_meta_map_.find(key) == topic_meta_map_.end(),
        "Duplicate topic meta, topic name: {}, msg type: {}.",
        topic_meta_option.topic_name, topic_meta_option.msg_type);

    uint32_t topic_id = metadata_.topics.size();
    TopicMeta topic_meta{
        .id = topic_id,
        .topic_name = topic_meta_option.topic_name,
        .msg_type = topic_meta_option.msg_type,
        .serialization_type = topic_meta_option.serialization_type};

    metadata_.topics.emplace_back(topic_meta);
    topic_meta_map_.emplace(key, topic_meta);
  }

  // misc
  max_bag_size_ = options_.storage_policy.max_bag_size_m * 1024 * 1024;
  max_preparation_duration_ns_ = options_.max_preparation_duration_s * 1000000000;

  sqlite3_config(SQLITE_CONFIG_SINGLETHREAD);

  options = options_;
}

void RecordAction::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void RecordAction::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  sync_timer_->Cancel();

  std::promise<void> stop_promise;
  executor_.Execute([this, &stop_promise]() {
    CloseDb();
    stop_promise.set_value();
  });
  stop_promise.get_future().wait();
}

void RecordAction::InitExecutor() {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  AIMRT_CHECK_ERROR_THROW(
      get_executor_func_,
      "Get executor function is not set before initialize.");

  executor_ = get_executor_func_(options_.timer_executor);
  AIMRT_CHECK_ERROR_THROW(
      executor_, "Can not get executor {}.", options_.timer_executor);
  AIMRT_CHECK_ERROR_THROW(
      executor_.ThreadSafe(), "Record executor {} is not thread safe!", options_.timer_executor);
}

void RecordAction::RegisterGetExecutorFunc(
    const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  get_executor_func_ = get_executor_func;
}

void RecordAction::RegisterGetTypeSupportFunc(
    const std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  get_type_support_func_ = get_type_support_func;
}

size_t RecordAction::GetDbFileSize() const {
  if (cur_db_file_path_.empty() || !std::filesystem::exists(cur_db_file_path_)) return 0;
  return std::filesystem::file_size(cur_db_file_path_);
}

void RecordAction::AddRecord(OneRecord&& record) {
  if (state_.load() != State::kStart) [[unlikely]] {
    return;
  }

  if (options_.mode == Options::Mode::kImd) {
    executor_.Execute([this, record{std::move(record)}]() mutable {
      if (state_.load() != State::kStart) [[unlikely]] {
        return;
      }

      AddRecordImpl(std::move(record));
    });
  } else if (options_.mode == Options::Mode::kSignal) {
    executor_.Execute([this, record{std::move(record)}]() mutable {
      if (state_.load() != State::kStart) [[unlikely]] {
        return;
      }

      uint64_t cur_timestamp = record.timestamp;

      if (recording_flag_) {
        if (stop_record_timestamp_ != 0 && cur_timestamp > stop_record_timestamp_) {
          recording_flag_ = false;
        }
      }

      if (recording_flag_) {
        AddRecordImpl(std::move(record));
      } else {
        cur_cache_.emplace_back(std::move(record));

        auto cur_cache_begin_timestamp = cur_cache_.begin()->timestamp;

        if ((cur_timestamp - cur_cache_begin_timestamp) > max_preparation_duration_ns_) {
          cur_cache_.swap(last_cache_);
          cur_cache_.clear();
        }
      }
    });
  }
}

bool RecordAction::StartSignalRecord(uint64_t preparation_duration_s, uint64_t record_duration_s) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kStart,
      "Method can only be called when state is 'Start'.");

  if (options_.mode != Options::Mode::kSignal) [[unlikely]] {
    AIMRT_WARN("Cur action mode is not signal mode.");
    return false;
  }

  if (recording_flag_) [[unlikely]] {
    AIMRT_WARN("Recording is already running.");
    return false;
  }

  executor_.Execute(
      [this, preparation_duration_s, record_duration_s]() {
        if (state_.load() != State::kStart) [[unlikely]] {
          return;
        }

        if (recording_flag_) {
          AIMRT_WARN("Recording is already in progress");
          return;
        }

        recording_flag_ = true;

        uint64_t now = aimrt::common::util::GetCurTimestampNs();

        stop_record_timestamp_ = now + record_duration_s * 1000000000;

        uint64_t start_record_timestamp = now - preparation_duration_s * 1000000000;

        // 二分查找到缓存中需要开始记录的地方
        size_t last_cache_size = last_cache_.size();
        size_t cur_cache_size = cur_cache_.size();
        size_t all_cache_size = last_cache_size + cur_cache_size;

        if (all_cache_size == 0) return;

        auto get_cahce_timestamp = [&](size_t idx) {
          return (idx < last_cache_size)
                     ? last_cache_[idx].timestamp
                     : cur_cache_[idx - last_cache_size].timestamp;
        };

        size_t low = 0, high = all_cache_size - 1;

        while (low < high) {
          size_t mid = low + (high - low) / 2;
          if (get_cahce_timestamp(mid) < start_record_timestamp)
            low = mid + 1;
          else
            high = mid;
        }

        // 将缓存写入db
        if (low < last_cache_size) {
          for (size_t ii = low; ii < last_cache_size; ++ii) {
            AddRecordImpl(std::move(last_cache_[ii]));
          }

          for (size_t ii = 0; ii < cur_cache_size; ++ii) {
            AddRecordImpl(std::move(cur_cache_[ii]));
          }
        } else {
          for (size_t ii = low - last_cache_size; ii < cur_cache_size; ++ii) {
            AddRecordImpl(std::move(cur_cache_[ii]));
          }
        }

        // 清空缓存
        last_cache_.clear();
        cur_cache_.clear();
      });

  return true;
}

void RecordAction::StopSignalRecord() {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kStart,
      "Method can only be called when state is 'Start'.");

  if (options_.mode != Options::Mode::kSignal) [[unlikely]] {
    AIMRT_WARN("Cur action mode is not signal mode.");
    return;
  }

  executor_.Execute([this]() { recording_flag_ = false; });
}

void RecordAction::AddRecordImpl(OneRecord&& record) {
  if (db_ == nullptr) [[unlikely]] {
    // first record
    OpenNewDb(record.timestamp);
  } else if (cur_data_size_ * estimated_overhead_ >= max_bag_size_) [[unlikely]] {
    size_t original_cur_data_size = cur_data_size_;
    cur_data_size_ = 0;
    estimated_overhead_ = std::max(1.0, static_cast<double>(GetDbFileSize()) / original_cur_data_size);
    OpenNewDb(record.timestamp);
  }

  if (cur_exec_count_ == 0) [[unlikely]] {
    sqlite3_exec(db_, "BEGIN", 0, 0, 0);
  }

  sqlite3_reset(insert_msg_stmt_);

  // insert data
  sqlite3_bind_int64(insert_msg_stmt_, 1, record.topic_index);
  sqlite3_bind_int64(insert_msg_stmt_, 2, record.timestamp);

  const auto& buffer_array_view = *record.buffer_view_ptr;

  if (buffer_array_view.Size() == 1) {
    auto data = buffer_array_view.Data()[0];
    sqlite3_bind_blob(insert_msg_stmt_, 3, data.data, data.len, SQLITE_STATIC);
    sqlite3_step(insert_msg_stmt_);

    buf_array_view_cache_.emplace_back(std::move(record.buffer_view_ptr));

    cur_data_size_ += data.len;
  } else {
    // TODO: is that safe?
    std::vector<char> buf = buffer_array_view.JoinToCharVector();
    sqlite3_bind_blob(insert_msg_stmt_, 3, buf.data(), buf.size(), SQLITE_STATIC);
    sqlite3_step(insert_msg_stmt_);

    buf_cache_.emplace_back(std::move(buf));

    cur_data_size_ += buf.size();
  }

  cur_data_size_ += 24;  // id + topic_id + timestamp
  ++cur_exec_count_;

  if (cur_exec_count_ >= options_.storage_policy.msg_write_interval) [[unlikely]] {
    cur_exec_count_ = 0;
    sqlite3_exec(db_, "COMMIT", 0, 0, 0);
    buf_array_view_cache_.clear();
    buf_cache_.clear();
  }
}

void RecordAction::OpenNewDb(uint64_t start_timestamp) {
  CloseDb();

  std::string cur_db_file_name = bag_base_name_ + "_" + std::to_string(cur_db_file_index_) + ".db3";

  cur_db_file_path_ = (real_bag_path_ / cur_db_file_name).string();

  ++cur_db_file_index_;

  // open db
  int ret = sqlite3_open(cur_db_file_path_.c_str(), &db_);
  AIMRT_CHECK_ERROR_THROW(ret == SQLITE_OK,
                          "Sqlite3 open db file failed, path: {}, ret: {}, error info: {}",
                          cur_db_file_path_, ret, sqlite3_errmsg(db_));

  AIMRT_TRACE("Open new db, path: {}", cur_db_file_path_);

  std::string journal_mode_sql = "PRAGMA journal_mode = " + options_.storage_policy.journal_mode + ";";
  sqlite3_exec(db_, journal_mode_sql.c_str(), 0, 0, 0);

  std::string synchronous_mode_sql = "PRAGMA synchronous = " + options_.storage_policy.synchronous_mode + ";";
  sqlite3_exec(db_, synchronous_mode_sql.c_str(), 0, 0, 0);

  // create table
  std::string sql = R"str(
CREATE TABLE messages(
id          INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
topic_id    INTEGER NOT NULL,
timestamp   INTEGER NOT NULL,
data        BLOB NOT NULL);
)str";

  ret = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, nullptr);
  AIMRT_CHECK_ERROR_THROW(ret == SQLITE_OK,
                          "Sqlite3 create table failed, sql: {}, ret: {}, error info: {}",
                          sql, ret, sqlite3_errmsg(db_));

  // create stmt
  sql = "INSERT INTO messages(topic_id, timestamp, data) VALUES(?, ?, ?)";

  ret = sqlite3_prepare_v3(db_, sql.c_str(), sql.size(), 0, &insert_msg_stmt_, nullptr);
  AIMRT_CHECK_ERROR_THROW(ret == SQLITE_OK,
                          "Sqlite3 prepare failed, sql: {}, ret: {}, error info: {}",
                          sql, ret, sqlite3_errmsg(db_));

  // update metadatat.yaml
  metadata_.files.emplace_back(
      MetaData::FileMeta{
          .path = cur_db_file_name,
          .start_timestamp = start_timestamp});

  // check and del db file
  if (options_.storage_policy.max_bag_num > 0 && metadata_.files.size() > options_.storage_policy.max_bag_num) {
    auto itr = metadata_.files.begin();
    std::filesystem::remove(real_bag_path_ / itr->path);
    metadata_.files.erase(itr);
  }

  YAML::Node node;
  node["aimrt_bagfile_information"] = metadata_;

  std::ofstream ofs(metadata_yaml_file_path_);
  ofs << node;
  ofs.close();
}

void RecordAction::CommitRecord(aimrt::executor::ExecutorRef& storage_executor_ref_) {
  auto timer_task = [this]() {
    executor_.Execute([this]() {
      if (db_ == nullptr)
        return;
      sqlite3_exec(db_, "COMMIT", 0, 0, 0);
      buf_array_view_cache_.clear();
      buf_cache_.clear();
      cur_exec_count_ = 0;
      sqlite3_exec(db_, "BEGIN", 0, 0, 0);
    });
  };
  sync_timer_ = executor::CreateTimer(storage_executor_ref_, std::chrono::milliseconds(options_.storage_policy.msg_write_interval_time), std::move(timer_task));
}

void RecordAction::CloseDb() {
  if (db_ != nullptr) {
    if (insert_msg_stmt_ != nullptr) {
      if (cur_exec_count_ > 0) {
        cur_exec_count_ = 0;
        sqlite3_exec(db_, "COMMIT", 0, 0, 0);
        buf_array_view_cache_.clear();
        buf_cache_.clear();
      }

      sqlite3_finalize(insert_msg_stmt_);
      insert_msg_stmt_ = nullptr;
    }

    sqlite3_close_v2(db_);
    db_ = nullptr;
  }
}
}  // namespace aimrt::plugins::record_playback_plugin