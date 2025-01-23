// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "record_playback_plugin/record_action.h"

#include <fstream>
#include <future>
#include <string>
#include <unordered_set>

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
    storage_policy["storage_format"] = rhs.storage_policy.storage_format;
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
    node["executor"] = rhs.executor;

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

      rhs.storage_policy.storage_format = storage_policy["storage_format"].as<std::string>();

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
        static const std::unordered_set<std::string> valid_journal_mode_set = {"delete", "truncate", "persist", "memory", "wal", "off"};

        if (!valid_journal_mode_set.contains(journal_mode)) {
          throw aimrt::common::util::AimRTException("Invalid journal mode: " + journal_mode);
        }
        rhs.storage_policy.journal_mode = journal_mode;
      }

      if (storage_policy["synchronous_mode"]) {
        auto synchronous_mode = aimrt::common::util::StrToLower(storage_policy["synchronous_mode"].as<std::string>());
        static const std::unordered_set<std::string> valid_synchronous_mode_set = {"off", "normal", "full", "extra"};

        if (!valid_synchronous_mode_set.contains(synchronous_mode)) {
          throw aimrt::common::util::AimRTException("Invalid synchronous mode: " + synchronous_mode);
        }
        rhs.storage_policy.synchronous_mode = synchronous_mode;
      }
    }

    if (node["max_preparation_duration_s"])
      rhs.max_preparation_duration_s = node["max_preparation_duration_s"].as<uint64_t>();

    rhs.executor = node["executor"].as<std::string>();

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

  // storage change

  printf("storage_policy.storage_format: %s", options_.storage_policy.storage_format.c_str());

  if (options_.storage_policy.storage_format == "sqlite") {
    storage_ = std::make_unique<SqliteStorage>();
  } else if (options_.storage_policy.storage_format == "mcap") {
    storage_ = std::make_unique<McapStorage>();
  } else {
    AIMRT_ERROR_THROW("storage format is not sqlite or mcap");
  }

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

  storage_->Initialize(options_.bag_path, max_bag_size_, metadata_, get_type_support_func_);

  // InitMcapRecord();

  options = options_;
}

void RecordAction::InitMcapRecord() {
  // auto options = mcap::McapWriterOptions("");
  // const auto res = writer_.open(options_.bag_path+"/ros2_test.mcap", options);
  // mcap::Header header{
  //   .profile="aimrt_profile",
  //   .library="aimrt_version",
  // };

  // ros2
  // auto tsf = get_type_support_func_(options_.topic_meta_list[0].msg_type);
  // auto ts_ptr = reinterpret_cast<const rosidl_message_type_support_t *>(tsf.CustomTypeSupportPtr());
  // const rosidl_message_type_support_t* specific_support =
  //     ts_ptr->func(ts_ptr, "rosidl_typesupport_introspection_cpp");
  //  if (specific_support) {
  //             const auto* type_data = static_cast<const MessageMembers*>(
  //                 specific_support->data);
  //           std::cout << "Build Ros2 Schema : \n \n"<<  BuildROS2Schema(type_data,0) << std::endl;

  //   } else {
  //     AIMRT_INFO("Build Failed");

  //   }
  // const auto* type_data = static_cast<const MessageMembers*>(
  //               specific_support->data);

  // std::string schema_string = std::string(type_data->message_namespace_).replace(std::string(type_data->message_namespace_).find("::"), 2, "/") + "/" + type_data->message_name_;  // schema 名称

  // mcap::Schema schema(
  //     schema_string,  // schema 名称
  //     "ros2msg",          // schema 编码格式
  //     BuildROS2Schema(type_data,0)          // schema 数据
  // );
  // writer_.addSchema(schema);
  // mcap::Channel channel(
  //     "/test_topic",        // topic 名称
  //     "cdr",         // 使用 schema 名称
  //     schema.id            // 关联的 schema ID
  // );
  // writer_.addChannel(channel);
  // channelId=channel.id;
  // std::cout << cst->func(cst, "rosidl_typesupport_introspection_cpp");

  // auto options = mcap::McapWriterOptions("");
  // const auto res = writer_.open(options_.bag_path+"/protobuf_test.mcap", options);

  // auto tsf = get_type_support_func_(options_.topic_meta_list[0].msg_type);
  // // 获取类型支持指针
  // auto ts_ptr = reinterpret_cast<const google::protobuf::Descriptor*>(tsf.CustomTypeSupportPtr());
  // AIMRT_INFO("开始转换: {}", BuildFileDescriptorSet(ts_ptr).SerializeAsString());

  // auto schema_data = BuildFileDescriptorSet(ts_ptr).SerializeAsString();
  // mcap::Schema schema(
  //     ts_ptr->full_name(),  // schema 名称
  //     "protobuf",          // schema 编码格式
  //     schema_data          // schema 数据
  // );
  // writer_.addSchema(schema);

  // mcap::Channel channel(
  //     "test_topic",        // topic 名称
  //     "protobuf",         // 使用 schema 名称
  //     schema.id            // 关联的 schema ID
  // );
  // writer_.addChannel(channel);
  // channelId=channel.id;
}

void RecordAction::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
  sync_timer_->Reset();
}

void RecordAction::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  std::promise<void> stop_promise;
  executor_.Execute([this, &stop_promise]() {
    CloseDb();
    stop_promise.set_value();
  });

  sync_timer_->Cancel();
  sync_timer_->SyncWait();

  stop_promise.get_future().wait();
}

void RecordAction::InitExecutor(aimrt::executor::ExecutorRef timer_executor) {
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
      executor_.ThreadSafe(), "Record executor {} is not thread safe!", options_.executor);

  auto timer_task = [this]() {
    executor_.Execute([this]() {
      if (db_ == nullptr)
        return;
      cur_exec_count_ = 1;  // avoid BEGIN again
      sqlite3_exec(db_, "COMMIT", 0, 0, 0);
      buf_array_view_cache_.clear();
      buf_cache_.clear();
      sqlite3_exec(db_, "BEGIN", 0, 0, 0);
    });
  };

  sync_timer_ = executor::CreateTimer(timer_executor, std::chrono::milliseconds(options_.storage_policy.msg_write_interval_time), std::move(timer_task), false);
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
  AIMRT_CHECK_ERROR(storage_, "storage_ is nullptr.");
  storage_->WriteRecord(record.topic_index, record.timestamp, record.buffer_view_ptr);
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