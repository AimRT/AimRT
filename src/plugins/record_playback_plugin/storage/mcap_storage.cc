// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mcap_storage.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/descriptor.pb.h"
#include "record_playback_plugin/global.h"

namespace aimrt::plugins::record_playback_plugin {

bool McapStorage::InitializeRecord(const std::string& bag_path, uint64_t max_bag_size, uint64_t max_bag_num,
                                   MetaData& metadata, std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func) {
  auto tm = aimrt::common::util::GetCurTm();
  char buf[17];  // _YYYYMMDD_hhmmss
  snprintf(buf, sizeof(buf), "_%04d%02d%02d_%02d%02d%02d",
           (tm.tm_year + 1900) % 10000u, (tm.tm_mon + 1) % 100u, (tm.tm_mday) % 100u,
           (tm.tm_hour) % 100u, (tm.tm_min) % 100u, (tm.tm_sec) % 100u);
  bag_base_name_ = "aimrtbag" + std::string(buf);
  std::filesystem::path parent_bag_path = std::filesystem::absolute(bag_path);
  if (!(std::filesystem::exists(parent_bag_path) && std::filesystem::is_directory(parent_bag_path))) {
    std::filesystem::create_directories(parent_bag_path);
  }

  real_bag_path_ = parent_bag_path / bag_base_name_;

  AIMRT_CHECK_ERROR_THROW(!std::filesystem::exists(real_bag_path_),
                          "Bag path '{}' does not exist!", real_bag_path_.string());

  std::filesystem::create_directories(real_bag_path_);

  metadata_yaml_file_path_ = real_bag_path_ / "metadata.yaml";
  metadata_ = metadata;
  max_bag_size_ = max_bag_size;
  max_bag_num_ = max_bag_num;

  get_type_support_func_ = get_type_support_func;
  AIMRT_CHECK_ERROR_THROW(get_type_support_func_, "get_type_support_func_ is nullptr!");

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
          mcap_struct{
              .schema_name = schema_name,
              .schema_format = "ros2msg",
              .schema_data = BuildROS2Schema(type_data, 0),
              .channel_name = topic_meta.topic_name,
              .channel_format = "cdr"});
    } else if (topic_meta.serialization_type == "pb") {
      auto ts_ptr = reinterpret_cast<const google::protobuf::Descriptor*>(type_support_ref.CustomTypeSupportPtr());
      mcap_info_map_.emplace(
          topic_meta.id,
          mcap_struct{
              .schema_name = ts_ptr->full_name(),
              .schema_format = "protobuf",
              .schema_data = BuildPbSchema(ts_ptr).SerializeAsString(),
              .channel_name = topic_meta.topic_name,
              .channel_format = "protobuf"});
    } else {
      AIMRT_WARN("Unsupported serialization type in mcap format: {}", topic_meta.serialization_type);
    }
  }
  return true;
}

void McapStorage::SetStoragePolicy(const std::string& compression_mode, const std::string& compression_level) {
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
  storage_policy_.compression_mode = compression_mode_map[compression_mode];
  storage_policy_.compression_level = compression_level_map[compression_level];
}

bool McapStorage::InitializePlayback(const std::string& bag_path, MetaData& metadata, uint64_t skip_duration_s, uint64_t play_duration_s) {
  real_bag_path_ = std::filesystem::absolute(bag_path);
  metadata_ = metadata;

  start_playback_timestamp_ = metadata_.files[0].start_timestamp + skip_duration_s * 1000000000;

  if (play_duration_s == 0) {
    stop_playback_timestamp_ = 0;
  } else {
    stop_playback_timestamp_ = start_playback_timestamp_ + play_duration_s * 1000000000;
  }

  size_t ii = 1;
  for (; ii < metadata.files.size(); ++ii) {
    if (metadata.files[ii].start_timestamp > start_playback_timestamp_) break;
  }
  cur_mcap_file_index_ = ii - 1;

  for (auto& topic_meta : metadata.topics) {
    topic_name_to_topic_id_map_[topic_meta.topic_name] = topic_meta.id;
    AIMRT_INFO("topic_name: {}, topic_id: {}", topic_meta.topic_name, topic_meta.id);
  }
  return true;
}

bool McapStorage::WriteRecord(uint64_t topic_id, uint64_t timestamp,
                              std::shared_ptr<aimrt::util::BufferArrayView> buffer_view_ptr) {
  if (cur_mcap_file_path_.empty() || !std::filesystem::exists(cur_mcap_file_path_)) [[unlikely]] {
    OpenNewStorageToRecord(timestamp);
  } else if (cur_data_size_ * estimated_overhead_ >= max_bag_size_) [[unlikely]] {
    size_t original_cur_data_size = cur_data_size_;
    cur_data_size_ = 0;
    estimated_overhead_ = std::max(0.1, static_cast<double>(GetFileSize()) / original_cur_data_size);
    AIMRT_INFO("estimated_overhead: {}, max_bag_size: {}, cur_data_size: {}", estimated_overhead_, max_bag_size_, original_cur_data_size);
    OpenNewStorageToRecord(timestamp);
  }

  mcap::Message msg{
      .channelId = topic_id_to_channel_id_map_[topic_id],
      .sequence = topic_id_to_seq_[topic_id]++,
      .logTime = timestamp,
      .publishTime = timestamp,  // 3.9.1 plogjuggler does not support logTime in mcap, so need to set publishTime
  };
  cur_data_size_ += 32;

  const auto& buffer_array_view = *buffer_view_ptr;
  if (buffer_array_view.Size() == 1) {
    auto data = buffer_array_view.Data()[0];
    msg.data = reinterpret_cast<const std::byte*>(data.data);
    msg.dataSize = data.len;
    cur_data_size_ += data.len;
  } else {
    auto data = buffer_array_view.JoinToString();
    msg.data = reinterpret_cast<const std::byte*>(data.data());
    msg.dataSize = data.size();
    cur_data_size_ += data.size();
  }

  auto res = writer_->write(msg);
  if (!res.ok()) {
    AIMRT_WARN("Failed to write record to mcap file: {}", res.message);
  }
  return true;
}

void McapStorage::FlushToDisk() {
  if (writer_)
    writer_->closeLastChunk();
}

bool McapStorage::ReadRecord(uint64_t& topic_id, uint64_t& timestamp, std::unique_ptr<char[]>& data, size_t& size) {
  if (!msg_reader_itr_ || *msg_reader_itr_ == msg_reader_ptr_->end()) {
    if (!OpenNewStorageToPlayback()) {
      AIMRT_WARN("OpenNewStorageToPlayback failed");
      return false;
    }
  }

  if (!msg_reader_itr_ || *msg_reader_itr_ == msg_reader_ptr_->end()) {
    AIMRT_INFO("msg_reader_itr_ is nullptr or end");
    return false;
  }

  std::unique_lock<std::mutex> lck(mcap_playback_mutex_);

  const auto& message = (**msg_reader_itr_).message;
  topic_id = channel_id_to_topic_id_map_[message.channelId];
  timestamp = message.logTime;
  size = message.dataSize;
  data = std::make_unique<char[]>(size);
  std::memcpy(data.get(), message.data, size);
  (*msg_reader_itr_)++;

  return true;
}

void McapStorage::CloseRecord() {
  if (writer_)
    writer_->close();
}

void McapStorage::ClosePlayback() {
  if (reader_) {
    reader_->close();
  }
  if (msg_reader_itr_) {
    msg_reader_itr_.reset();
  }
  if (msg_reader_ptr_) {
    msg_reader_ptr_.reset();
  }
}

size_t McapStorage::GetFileSize() const {
  if (cur_mcap_file_path_.empty() || !std::filesystem::exists(cur_mcap_file_path_)) return 0;
  return std::filesystem::file_size(cur_mcap_file_path_);
}

void McapStorage::OpenNewStorageToRecord(uint64_t start_timestamp) {
  CloseRecord();
  writer_ = std::make_unique<mcap::McapWriter>();
  std::string cur_mcap_file_name = bag_base_name_ + "_" + std::to_string(cur_mcap_file_index_) + ".mcap";
  cur_mcap_file_path_ = (real_bag_path_ / cur_mcap_file_name).string();
  cur_mcap_file_index_++;

  auto options = mcap::McapWriterOptions("aimrtbag");
  options.compression = storage_policy_.compression_mode;
  options.compressionLevel = storage_policy_.compression_level;
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
          .start_timestamp = start_timestamp});

  // check and del record file
  if (max_bag_num_ > 0 && metadata_.files.size() > max_bag_num_) {
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

bool McapStorage::OpenNewStorageToPlayback() {
  ClosePlayback();
  if (cur_mcap_file_index_ >= metadata_.files.size()) [[unlikely]] {
    AIMRT_INFO("cur_map_file_index_: {}, ALL files : {}, there is no more record file", cur_mcap_file_index_, metadata_.files.size());
    return false;
  }

  const auto& mcap_file_meta = metadata_.files[cur_mcap_file_index_];
  const auto mcap_file_path = (real_bag_path_ / mcap_file_meta.path).string();
  ++cur_mcap_file_index_;

  uint64_t cur_mcap_start_timestamp = mcap_file_meta.start_timestamp;

  if (stop_playback_timestamp_ && cur_mcap_start_timestamp > stop_playback_timestamp_) [[unlikely]] {
    AIMRT_INFO("cur_db_start_timestamp: {}, stop_playback_timestamp: {}", cur_mcap_start_timestamp, stop_playback_timestamp_);
    return false;
  }

  reader_ = std::make_unique<mcap::McapReader>();

  auto ret = reader_->open(mcap_file_path);
  if (!ret.ok()) [[unlikely]] {
    AIMRT_ERROR("open mcap file {} failed, error: {}", mcap_file_path, ret.message);
    return false;
  }

  // when summary is empty, scan the whole file
  auto summary = reader_->readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  const auto& channels = reader_->channels();

  if (channels.empty()) {
    AIMRT_ERROR("No channels found in mcap file: {}", mcap_file_path);
    return false;
  }

  for (auto& [channel_id, channel] : reader_->channels()) {
    auto iter = topic_name_to_topic_id_map_.find(channel->topic);
    if (iter == topic_name_to_topic_id_map_.end()) continue;
    channel_id_to_topic_id_map_[channel_id] = topic_name_to_topic_id_map_[channel->topic];
  }

  mcap::ReadMessageOptions options;

  options.startTime = start_playback_timestamp_;

  if (stop_playback_timestamp_ > 0) {
    AIMRT_INFO("stop_playback_timestamp: {}", stop_playback_timestamp_);
    options.endTime = stop_playback_timestamp_;
  } else {
    options.endTime = mcap::MaxTime;
  }
  options.topicFilter = [&](std::string_view topic_name) {
    if (topic_name_to_topic_id_map_.find(std::string(topic_name)) != topic_name_to_topic_id_map_.end()) {
      return true;
    }
    return false;
  };

  msg_reader_ptr_ = std::make_unique<mcap::LinearMessageView>(reader_->readMessages([this](const mcap::Status& status) {
    AIMRT_INFO("mcap readMessages failed : {}", status.message);
  },
                                                                                    options));

  msg_reader_itr_ = std::make_unique<mcap::LinearMessageView::Iterator>(msg_reader_ptr_->begin());

  return true;
}

google::protobuf::FileDescriptorSet McapStorage::BuildPbSchema(const google::protobuf::Descriptor* toplevel_descriptor) {
  google::protobuf::FileDescriptorSet fd_set;
  std::queue<const google::protobuf::FileDescriptor*> to_add;
  to_add.push(toplevel_descriptor->file());
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

std::string McapStorage::BuildROS2Schema(const MessageMembers* members, int indent = 0) {
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