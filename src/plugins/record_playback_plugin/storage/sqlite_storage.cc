// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "sqlite_storage.h"

namespace aimrt::plugins::record_playback_plugin {

bool SqliteStorage::Initialize(const std::string& bag_path, uint64_t max_bag_size, MetaData& metadata,
                               std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func_) {
  // bag_path
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
                          "Bag path '{}' is exist!", real_bag_path_.string());

  std::filesystem::create_directories(real_bag_path_);

  metadata_yaml_file_path_ = real_bag_path_ / "metadata.yaml";
  metadata_.version = kVersion;
  metadata_ = metadata;
  max_bag_size = max_bag_size;

  // sqlite3 config
  sqlite3_config(SQLITE_CONFIG_SINGLETHREAD);

  return true;
}

bool SqliteStorage::ReadRecord(uint64_t& stop_playback_timestamp, uint64_t& timestamp,
                               void*& data, size_t& size) {

  return true;
}

size_t SqliteStorage::GetFileSize() const {
  if (cur_db_file_path_.empty() || !std::filesystem::exists(cur_db_file_path_)) return 0;
  return std::filesystem::file_size(cur_db_file_path_);
}

bool SqliteStorage::WriteRecord(uint64_t timestamp, uint64_t topic_index, std::shared_ptr<aimrt::util::BufferArrayView> buffer_view_ptr) {
  if (db_ == nullptr) [[unlikely]] {
    // first record
    OpenNewStorageToRecord(timestamp);
  } else if (cur_data_size_ * estimated_overhead_ >= max_bag_size_) [[unlikely]] {
    size_t original_cur_data_size = cur_data_size_;
    cur_data_size_ = 0;
    estimated_overhead_ = std::max(1.0, static_cast<double>(GetFileSize()) / original_cur_data_size);
    AIMRT_INFO("estimated_overhead: {}, max_bag_size: {}, cur_data_size: {}", estimated_overhead_, max_bag_size_, original_cur_data_size);
    OpenNewStorageToRecord(timestamp);
  }

  if (cur_exec_count_ == 0) [[unlikely]] {
    sqlite3_exec(db_, "BEGIN", 0, 0, 0);
  }

  sqlite3_reset(insert_msg_stmt_);

  // insert data
  sqlite3_bind_int64(insert_msg_stmt_, 1, topic_index);
  sqlite3_bind_int64(insert_msg_stmt_, 2, timestamp);

  const auto& buffer_array_view = *buffer_view_ptr;

  if (buffer_array_view.Size() == 1) {
    auto data = buffer_array_view.Data()[0];
    sqlite3_bind_blob(insert_msg_stmt_, 3, data.data, data.len, SQLITE_STATIC);
    sqlite3_step(insert_msg_stmt_);

    buf_array_view_cache_.emplace_back(std::move(buffer_view_ptr));

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

  if (cur_exec_count_ >= storage_policy.msg_write_interval) [[unlikely]] {
    cur_exec_count_ = 0;
    sqlite3_exec(db_, "COMMIT", 0, 0, 0);
    buf_array_view_cache_.clear();
    buf_cache_.clear();
  }

  return true;
}

void SqliteStorage::OpenNewStorageToRecord(uint64_t start_timestamp) {
  Close();

  std::string cur_db_file_name = bag_base_name_ + "_" + std::to_string(cur_db_file_index_) + ".db3";

  cur_db_file_path_ = (real_bag_path_ / cur_db_file_name).string();

  ++cur_db_file_index_;

  // open db
  int ret = sqlite3_open(cur_db_file_path_.c_str(), &db_);
  AIMRT_CHECK_ERROR_THROW(ret == SQLITE_OK,
                          "Sqlite3 open db file failed, path: {}, ret: {}, error info: {}",
                          cur_db_file_path_, ret, sqlite3_errmsg(db_));

  AIMRT_TRACE("Open new db, path: {}", cur_db_file_path_);

  std::string journal_mode_sql = "PRAGMA journal_mode = " + storage_policy.journal_mode + ";";
  sqlite3_exec(db_, journal_mode_sql.c_str(), 0, 0, 0);

  std::string synchronous_mode_sql = "PRAGMA synchronous = " + storage_policy.synchronous_mode + ";";
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
  if (storage_policy.max_bag_num > 0 && metadata_.files.size() > storage_policy.max_bag_num) {
    auto itr = metadata_.files.begin();
    std::filesystem::remove(real_bag_path_ / itr->path);
    metadata_.files.erase(itr);
  }

  YAML::Node node;
  node["aimrt_bagfile_information"] = metadata_;

  std::ofstream ofs(metadata_yaml_file_path_);
  ofs << node;
  ofs.close();

  return;
}

void SqliteStorage::Close() {
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