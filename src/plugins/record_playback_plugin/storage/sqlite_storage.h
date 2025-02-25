// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <deque>
#include <vector>

#include "sqlite3.h"
#include "storage_interface.h"

namespace aimrt::plugins::record_playback_plugin {

class SqliteStorage : public StorageInterface {
 public:
  bool InitializeRecord(const std::string& path, uint64_t max_bag_size_, uint64_t max_bag_num, MetaData& metadata,
                        std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func_) override;

  bool InitializePlayback(const std::string& bag_path, MetaData& metadata, uint64_t skip_duration_s, uint64_t play_duration_s) override;

  bool ReadRecord(uint64_t& topic_id, uint64_t& timestamp, std::unique_ptr<char[]>& data, size_t& size) override;

  bool WriteRecord(uint64_t topic_id, uint64_t timestamp,
                   std::shared_ptr<aimrt::util::BufferArrayView> buffer_view_ptr) override;

  void FlushToDisk() override;

  void CloseRecord() override;
  void ClosePlayback() override;

  void SetStoragePolicy(const std::string& journal_mode, const std::string& synchronous_mode);

 private:
  size_t GetFileSize() const;
  void OpenNewStorageToRecord(uint64_t start_timestamp);
  bool OpenNewStorageToPlayback();
  void CloseDb();

 private:
  struct {
    std::string journal_mode;
    std::string synchronous_mode;
  } storage_policy_;

  std::deque<std::shared_ptr<aimrt::util::BufferArrayView>> buf_array_view_cache_;
  std::deque<std::vector<char>> buf_cache_;

  sqlite3* db_ = nullptr;
  sqlite3_stmt* insert_msg_stmt_ = nullptr;
  sqlite3_stmt* select_msg_stmt_ = nullptr;

  std::string db_path_;
  std::string cur_db_file_path_;
  std::string cur_db_file_name;

  std::string select_topic_id_;

  uint32_t cur_db_file_index_ = 0;
  uint64_t start_playback_timestamp_;
  uint64_t stop_playback_timestamp_;
  double estimated_overhead_ = 1.5;
  size_t cur_data_size_;
};

}  // namespace aimrt::plugins::record_playback_plugin