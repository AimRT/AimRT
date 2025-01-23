// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <deque>
#include <vector>

#include "record_playback_plugin/global.h"
#include "sqlite3.h"
#include "storage_interface.h"

namespace aimrt::plugins::record_playback_plugin {

class SqliteStorage : public StorageInterface {
 public:
  bool Initialize(const std::string& path, uint64_t max_bag_size_, MetaData& metadata,
                  std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func_) override;

  bool ReadRecord(uint64_t& topic_id, uint64_t& timestamp,
                  void*& data, size_t& size) override;

  bool WriteRecord(uint64_t topic_id, uint64_t timestamp,
                   std::shared_ptr<aimrt::util::BufferArrayView> buffer_view_ptr) override;

  void Close() override;
  size_t GetFileSize() const override;

 private:
  void OpenNewStorage(uint64_t start_timestamp) override;
  void CloseDb();

 private:
 private:
  size_t cur_exec_count_ = 0;
  sqlite3* db_ = nullptr;
  std::deque<std::shared_ptr<aimrt::util::BufferArrayView>> buf_array_view_cache_;
  std::deque<std::vector<char>> buf_cache_;
  sqlite3_stmt* insert_msg_stmt_ = nullptr;
  sqlite3_stmt* read_stmt_ = nullptr;
  std::string db_path_;
  std::string cur_db_file_path_;
  std::string cur_db_file_name;

  uint32_t cur_db_file_index_ = 0;
  double estimated_overhead_ = 1.5;
  size_t cur_data_size_;
};

}  // namespace aimrt::plugins::record_playback_plugin