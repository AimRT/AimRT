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

    Node storage_policy = YAML::Node();
    storage_policy["max_bag_size_m"] = rhs.storage_policy.max_bag_size_m;
    storage_policy["max_bag_num"] = rhs.storage_policy.max_bag_num;
    storage_policy["msg_write_interval"] = rhs.storage_policy.msg_write_interval;
    storage_policy["msg_write_interval_time"] = rhs.storage_policy.msg_write_interval_time;
    storage_policy["compression_mode"] = rhs.storage_policy.compression_mode;
    storage_policy["compression_level"] = rhs.storage_policy.compression_level;

    node["storage_policy"] = storage_policy;

    node["extra_attributes"] = rhs.extra_attributes;

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

      if (storage_policy["max_bag_size_m"])
        rhs.storage_policy.max_bag_size_m = storage_policy["max_bag_size_m"].as<uint32_t>();

      if (storage_policy["max_bag_num"])
        rhs.storage_policy.max_bag_num = storage_policy["max_bag_num"].as<uint32_t>();

      if (storage_policy["msg_write_interval"])
        rhs.storage_policy.msg_write_interval = storage_policy["msg_write_interval"].as<uint32_t>();

      if (storage_policy["msg_write_interval_time"])
        rhs.storage_policy.msg_write_interval_time = storage_policy["msg_write_interval_time"].as<uint32_t>();

      if (storage_policy["compression_mode"]) {
        auto compression_mode = aimrt::common::util::StrToLower(storage_policy["compression_mode"].as<std::string>());
        static const std::unordered_set<std::string> valid_compression_mode_set = {"none", "lz4", "zstd"};
        if (!valid_compression_mode_set.contains(compression_mode)) {
          throw aimrt::common::util::AimRTException("Invalid compression mode: " + compression_mode);
        }
        rhs.storage_policy.compression_mode = compression_mode;
      }

      if (storage_policy["compression_level"]) {
        auto compression_level = storage_policy["compression_level"].as<std::string>();
        static const std::unordered_set<std::string> valid_compression_level_set = {"fastest", "fast", "default", "slow", "slowest"};
        if (!valid_compression_level_set.contains(compression_level)) {
          throw aimrt::common::util::AimRTException("Invalid compression level: " + compression_level);
        }
        rhs.storage_policy.compression_level = compression_level;
      }
    }

    if (node["extra_attributes"] && node["extra_attributes"].IsMap()) {
      rhs.extra_attributes = node["extra_attributes"];
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

  // check topic meta
  for (auto& topic_meta : options_.topic_meta_list) {
    // check msg type
    auto type_support_ref = get_type_support_func_(topic_meta.msg_type);
    AIMRT_CHECK_ERROR_THROW(type_support_ref,
                            "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);

    // check serialization type
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

  metadata_.version = kVersion;
  if (options_.extra_attributes && !options_.extra_attributes.IsNull()) {
    metadata_.extra_attributes = options_.extra_attributes;
  } else {
    metadata_.extra_attributes = YAML::Node(YAML::NodeType::Null);
  }

  // storage change
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

  real_bag_path_ = parent_bag_path / bag_base_name_;

  AIMRT_CHECK_ERROR_THROW(!std::filesystem::exists(real_bag_path_),
                          "Bag path '{}' does not exist!", real_bag_path_.string());

  std::filesystem::create_directories(real_bag_path_);

  metadata_yaml_file_path_ = real_bag_path_ / "metadata.yaml";

  SetMcapOptions();
  for (auto& topic_meta : metadata_.topics) {
    auto type_support_ref = get_type_support_func_(topic_meta.msg_type);
    if (topic_meta.serialization_type == "ros2") {
      auto ts_ptr = reinterpret_cast<const rosidl_message_type_support_t*>(type_support_ref.CustomTypeSupportPtr());
      const rosidl_message_type_support_t* specific_support = ts_ptr->func(ts_ptr, "rosidl_typesupport_introspection_cpp");
      if (!specific_support) {
        AIMRT_WARN("Failed to get specific support for type support {} , maybe need to source specific topic msg_type", topic_meta.msg_type);
      }
      const auto* type_data = static_cast<const MessageMembers*>(
          specific_support->data);
      std::string schema_name = std::string(type_data->message_namespace_).replace(std::string(type_data->message_namespace_).find("::"), 2, "/") + "/" + type_data->message_name_;
      std::string schema_format = "ros2msg";
      mcap_info_map_.emplace(
          topic_meta.id,
          McapStruct{
              .schema_name = schema_name,
              .schema_format = "ros2msg",
              .schema_data = BuildROS2Schema(type_data, 0),
              .channel_name = topic_meta.topic_name,
              .channel_format = "cdr"});
    } else if (topic_meta.serialization_type == "pb") {
      auto ts_ptr = reinterpret_cast<const google::protobuf::Descriptor*>(type_support_ref.CustomTypeSupportPtr());
      mcap_info_map_.emplace(
          topic_meta.id,
          McapStruct{
              .schema_name = ts_ptr->full_name(),
              .schema_format = "protobuf",
              .schema_data = BuildPbSchema(ts_ptr).SerializeAsString(),
              .channel_name = topic_meta.topic_name,
              .channel_format = "protobuf"});
    } else {
      AIMRT_WARN("Unsupported serialization type in mcap format: {}", topic_meta.serialization_type);
    }
  }
  options = options_;
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
    CloseRecord();
    stop_promise.set_value();
  });

  sync_timer_->Cancel();

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
      FlushToDisk();
    });
  };

  sync_timer_ = executor::CreateTimer(timer_executor, std::chrono::milliseconds(options_.storage_policy.msg_write_interval_time), std::move(timer_task), false);
}

void RecordAction::FlushToDisk() {
  if (writer_) {
    writer_->closeLastChunk();
    cur_exec_count_ = 0;
  }
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

        // Use binary search to locate the starting point for recording in the cache
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

        // Write cache to db
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

        // Clear the cache
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

void RecordAction::UpdateMetadata(std::unordered_map<std::string, std::string>&& kv_pairs) {
  executor_.Execute([this, move_kv_pairs = std::move(kv_pairs)] {
  if (!metadata_.extra_attributes.IsMap()) {
    metadata_.extra_attributes = YAML::Node(YAML::NodeType::Map);
  }
  for (const auto& [key, value] : move_kv_pairs) {
    try {
      YAML::Node parsed_node = YAML::Load(value);
      metadata_.extra_attributes[key] = parsed_node;
    } catch (const std::exception& e) {
      metadata_.extra_attributes[key] = value;
    }
  }
  YAML::Node node;
  node["aimrt_bagfile_information"] = metadata_;

  std::ofstream ofs(metadata_yaml_file_path_);
  ofs << node;
  ofs.close(); });
}

size_t RecordAction::GetFileSize() const {
  if (cur_mcap_file_path_.empty() || !std::filesystem::exists(cur_mcap_file_path_)) return 0;
  return std::filesystem::file_size(cur_mcap_file_path_);
}

void RecordAction::AddRecordImpl(OneRecord&& record) {
  if (cur_mcap_file_path_.empty() || !std::filesystem::exists(cur_mcap_file_path_)) [[unlikely]] {
    OpenNewMcapToRecord(record.timestamp);
  } else if (cur_data_size_ * estimated_overhead_ >= max_bag_size_) [[unlikely]] {
    size_t original_cur_data_size = cur_data_size_;
    cur_data_size_ = 0;
    estimated_overhead_ = std::max(0.1, static_cast<double>(GetFileSize()) / original_cur_data_size);
    AIMRT_INFO("estimated_overhead: {}, max_bag_size: {}, cur_data_size: {}", estimated_overhead_, max_bag_size_, original_cur_data_size);
    OpenNewMcapToRecord(record.timestamp);
  }

  mcap::Message msg{
      .channelId = topic_id_to_channel_id_map_[record.topic_index],
      .sequence = 0,  // cant determine sequence, so set to 0
      .logTime = record.timestamp,
      .publishTime = record.timestamp,  // 3.9.1 plogjuggler does not support logTime in mcap, so need to set publishTime
  };

  auto data = record.buffer_view_ptr->JoinToString();
  msg.data = reinterpret_cast<const std::byte*>(data.data());
  msg.dataSize = data.size();
  cur_data_size_ += 30 + data.size();

  auto res = writer_->write(msg);
  if (!res.ok()) {
    AIMRT_WARN("Failed to write record to mcap file: {}", res.message);
  }

  cur_exec_count_++;
  if (cur_exec_count_ > options_.storage_policy.msg_write_interval) {
    FlushToDisk();
  }
}

void RecordAction::CloseRecord() {
  if (writer_)
    writer_->close();
}

void RecordAction::SetMcapOptions() {
  static std::unordered_map<std::string, mcap::Compression> compression_mode_map = {
      {"none", mcap::Compression::None},
      {"zstd", mcap::Compression::Zstd},
      {"lz4", mcap::Compression::Lz4},
  };
  static std::unordered_map<std::string, mcap::CompressionLevel> compression_level_map = {
      {"fastest", mcap::CompressionLevel::Fastest},
      {"fast", mcap::CompressionLevel::Fast},
      {"default", mcap::CompressionLevel::Default},
      {"slow", mcap::CompressionLevel::Slow},
      {"slowest", mcap::CompressionLevel::Slowest},
  };
  mcap_options.compression_mode = compression_mode_map[options_.storage_policy.compression_mode];
  mcap_options.compression_level = compression_level_map[options_.storage_policy.compression_level];
}

void RecordAction::OpenNewMcapToRecord(uint64_t timestamp) {
  CloseRecord();
  writer_ = std::make_unique<mcap::McapWriter>();
  std::string cur_mcap_file_name = bag_base_name_ + "_" + std::to_string(cur_mcap_file_index_) + ".mcap";
  cur_mcap_file_path_ = (real_bag_path_ / cur_mcap_file_name).string();
  cur_mcap_file_index_++;

  auto options = mcap::McapWriterOptions("aimrtbag");

  // setup compression mode and level

  options.compression = mcap_options.compression_mode;
  options.compressionLevel = mcap_options.compression_level;

  const auto res = writer_->open(cur_mcap_file_path_, options);
  if (!res.ok()) {
    AIMRT_ERROR("Failed to open mcap file '{}': {}", cur_mcap_file_path_, res.message);
    cur_mcap_file_path_.clear();
    return;
  }

  for (auto& [idx, mcap_info] : mcap_info_map_) {
    mcap::Schema schema(
        mcap_info.schema_name,
        mcap_info.schema_format,
        mcap_info.schema_data);
    writer_->addSchema(schema);

    mcap::Channel channel(
        mcap_info.channel_name,
        mcap_info.channel_format,
        schema.id);
    writer_->addChannel(channel);
    topic_id_to_channel_id_map_[idx] = channel.id;
  }

  metadata_.files.emplace_back(
      MetaData::FileMeta{
          .path = cur_mcap_file_name,
          .start_timestamp = timestamp});
  // check and del record file
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

google::protobuf::FileDescriptorSet RecordAction::BuildPbSchema(const google::protobuf::Descriptor* toplevelDescriptor) {
  google::protobuf::FileDescriptorSet fd_set;
  std::queue<const google::protobuf::FileDescriptor*> to_add;
  to_add.push(toplevelDescriptor->file());
  std::unordered_set<std::string> seen_dependencies;
  while (!to_add.empty()) {
    const google::protobuf::FileDescriptor* next_fd = to_add.front();
    to_add.pop();
    next_fd->CopyTo(fd_set.add_file());
    for (int i = 0; i < next_fd->dependency_count(); ++i) {
      const auto& dep = next_fd->dependency(i);
      if (seen_dependencies.find(dep->name()) == seen_dependencies.end()) {
        seen_dependencies.insert(dep->name());
        to_add.push(dep);
      }
    }
  }
  return fd_set;
}

std::string RecordAction::BuildROS2Schema(const MessageMembers* members, int indent = 0) {
  std::stringstream schema;
  std::queue<std::pair<const MessageMembers*, int>> queue;
  std::unordered_set<const MessageMembers*> visited;

  queue.push({members, indent});
  visited.insert(members);

  static auto appendArrayNotation = [](std::stringstream& ss, bool isArray) {
    if (isArray) {
      ss << "[] ";
    } else {
      ss << " ";
    }
  };

  static auto RosSchemaFormat = [](const MessageMembers* members) {
    std::string ns(members->message_namespace_);
    if (auto pos = ns.find("::"); pos != std::string::npos)
      ns.replace(pos, 2, "/");

    std::string schema_string = ns + "/" + members->message_name_;
    if (auto pos = schema_string.find("/msg"); pos != std::string::npos)
      schema_string.replace(pos, 4, "");

    return schema_string;
  };

  while (!queue.empty()) {
    auto [current_members, current_indent] = queue.front();
    queue.pop();

    AIMRT_CHECK_ERROR_THROW(current_indent <= 50, "Reached max recursion depth to resolve the schema");

    if (current_indent != 0) {
      schema << "================================================================================\n";
      schema << "MSG: " << RosSchemaFormat(current_members) << "\n";
    }

    for (size_t i = 0; i < current_members->member_count_; ++i) {
      const auto& member = current_members->members_[i];

      if (member.type_id_ == ROS_TYPE_MESSAGE) {
        const auto* nested_members = static_cast<const MessageMembers*>(member.members_->data);
        if (nested_members) {
          schema << RosSchemaFormat(nested_members);
          appendArrayNotation(schema, member.is_array_);
          schema << member.name_ << "\n";
          if (visited.find(nested_members) == visited.end()) {
            queue.push({nested_members, current_indent + 1});
            visited.insert(nested_members);
          }
        }
        continue;
      }

      switch (member.type_id_) {
        case ROS_TYPE_FLOAT:
          schema << "float32";
          break;
        case ROS_TYPE_DOUBLE:
          schema << "float64";
          break;
        case ROS_TYPE_CHAR:
          schema << "char";
          break;
        case ROS_TYPE_BOOL:
          schema << "bool";
          break;
        case ROS_TYPE_BYTE:
          schema << "byte";
          break;
        case ROS_TYPE_UINT8:
          schema << "uint8";
          break;
        case ROS_TYPE_INT8:
          schema << "int8";
          break;
        case ROS_TYPE_UINT16:
          schema << "uint16";
          break;
        case ROS_TYPE_INT16:
          schema << "int16";
          break;
        case ROS_TYPE_UINT32:
          schema << "uint32";
          break;
        case ROS_TYPE_INT32:
          schema << "int32";
          break;
        case ROS_TYPE_UINT64:
          schema << "uint64";
          break;
        case ROS_TYPE_INT64:
          schema << "int64";
          break;
        case ROS_TYPE_STRING:
          schema << "string";
          break;
      }
      appendArrayNotation(schema, member.is_array_);
      schema << member.name_ << "\n";
    }
  }

  return schema.str();
}

}  // namespace aimrt::plugins::record_playback_plugin