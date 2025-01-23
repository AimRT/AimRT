// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include "mcap/mcap.hpp"
#include "ros_storage.h"
#include "storage_interface.h"

namespace aimrt::plugins::record_playback_plugin {

class McapStorage : public StorageInterface {
 public:
  bool Initialize(const std::string& bag_path, uint64_t max_bag_size_, MetaData& metadata,
                  std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func_) override;
  bool WriteRecord(uint64_t topic_id, uint64_t timestamp,
                   std::shared_ptr<aimrt::util::BufferArrayView> buffer_view_ptr) override;
  bool ReadRecord(uint64_t& topic_id, uint64_t& timestamp,
                  void*& data, size_t& size) override;
  void Close() override;
  size_t GetFileSize() const override;

  void OpenNewStorage(uint64_t start_timestamp) override;

 private:
  std::string BuildROS2Schema(const MessageMembers* members, int indent);
  google::protobuf::FileDescriptorSet BuildPbSchema(const google::protobuf::Descriptor* toplevelDescriptor);

  std::function<aimrt::util::TypeSupportRef(std::string_view)> get_type_support_func_;

 private:
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
  std::unordered_map<uint64_t, mcap_struct> mcap_map_;
  std::unordered_map<uint64_t, unsigned short> topic_id_to_channel_id_map_;
  std::unordered_map<uint64_t, uint32_t> topic_id_to_cnt_;

  uint64_t max_bag_size_;
  size_t cur_data_size_;
  double estimated_overhead_ = 1.5;
  uint32_t sequence_cnt = 0;

  // std::unordered_map<
};

}  // namespace aimrt::plugins::record_playback_plugin