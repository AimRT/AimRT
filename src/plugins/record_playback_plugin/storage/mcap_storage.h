// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <mcap/mcap.hpp>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include "ros_storage.h"
#include "storage_interface.h"

namespace aimrt::plugins::record_playback_plugin {

class McapStorage : public StorageInterface {
 public:
  bool InitializeRecord(const std::string& bag_path, uint64_t max_bag_size_, uint64_t max_bag_num, MetaData& metadata,
                        std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func_) override;

  bool InitializePlayback(const std::string& bag_path, MetaData& metadata, uint64_t skip_duration_s, uint64_t play_duration_s) override;

  bool WriteRecord(uint64_t topic_id, uint64_t timestamp,
                   std::shared_ptr<aimrt::util::BufferArrayView> buffer_view_ptr) override;

  void FlushToDisk() override;

  bool ReadRecord(uint64_t& start_playback_timestamp, uint64_t& stop_playback_timestamp,
                  uint64_t& topic_id, uint64_t& timestamp, std::unique_ptr<char[]>& data, size_t& size) override;

  void CloseRecord() override;

  void ClosePlayback() override;

 private:
  size_t GetFileSize() const;

  void OpenNewStorageToRecord(uint64_t start_timestamp);

  bool OpenNewStorageToPlayback(uint64_t start_playback_timestamp, uint64_t stop_playback_timestamp);

  std::string BuildROS2Schema(const MessageMembers* members, int indent);
  google::protobuf::FileDescriptorSet BuildPbSchema(const google::protobuf::Descriptor* toplevelDescriptor);

  std::function<aimrt::util::TypeSupportRef(std::string_view)> get_type_support_func_;

 private:
  mcap::McapReader reader_;
  mcap::McapWriter writer_;
  std::string file_path_;
  std::string cur_mcap_file_path_;
  uint32_t cur_mcap_file_index_ = 0;

  struct mcap_struct {
    std::string schema_name;
    std::string schema_format;
    std::string schema_data;
    std::string channel_name;
    std::string channel_format;
  };
  std::unordered_map<uint64_t, mcap_struct> mcap_info_map_;  // use to record
  std::unordered_map<uint64_t, unsigned short> topic_id_to_channel_id_map_;
  std::unordered_map<uint64_t, uint32_t> topic_id_to_seq_;

  std::unordered_map<std::string, uint64_t> topic_name_to_topic_id_map_;  // use to playback
  std::unordered_map<uint64_t, unsigned short> channel_id_to_topic_id_map_;

  std::mutex mcap_mutex_;
  std::unique_ptr<mcap::LinearMessageView> msg_reader_ptr_;
  std::optional<mcap::LinearMessageView::Iterator> msg_reader_itr_;

  uint64_t start_playback_timestamp_;
  uint64_t stop_playback_timestamp_;

  size_t cur_data_size_;
  double estimated_overhead_ = 1.5;
  uint32_t sequence_cnt = 0;
};

}  // namespace aimrt::plugins::record_playback_plugin