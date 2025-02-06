// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "sqlite_storage.h"
#include "record_playback_plugin/global.h"

namespace aimrt::plugins::record_playback_plugin {

bool SqliteStorage::InitializeRecord(const std::string& bag_path, uint64_t max_bag_size, uint64_t max_bag_num, MetaData& metadata,
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
  metadata_ = metadata;
  max_bag_size_ = max_bag_size;
  max_bag_num_ = max_bag_num;
  AIMRT_INFO("Initialize record storage, bag_path: {}, max_bag_size: {}, max_bag_num: {}", bag_path,max_bag_size, max_bag_num);

  // sqlite3 config
  sqlite3_config(SQLITE_CONFIG_SINGLETHREAD);

  return true;
}

void SqliteStorage::SetStoragePolicy(const std::string& journal_mode, const std::string& synchronous_mode){
  storage_policy_.journal_mode = journal_mode;
  storage_policy_.synchronous_mode = synchronous_mode;
}

bool SqliteStorage::InitializePlayback(const std::string& bag_path, MetaData& metadata, uint64_t skip_duration_s, uint64_t play_duration_s, std::string select_topic_id) {
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
  cur_db_file_index_ = ii - 1;

  AIMRT_INFO("Start a new playback, skip_duration_s: {}, play_duration_s: {}, start_playback_timestamp: {}, stop_playback_timestamp: {}, use db index: {}",
             skip_duration_s, play_duration_s,
             start_playback_timestamp_, stop_playback_timestamp_,
             cur_db_file_index_);

  std::filesystem::path parent_bag_path = std::filesystem::absolute(bag_path);
  real_bag_path_ = parent_bag_path;
  metadata_ = metadata;
  select_topic_id_ = select_topic_id;

  return true;
}

bool SqliteStorage::ReadRecord(uint64_t& start_playback_timestamp, uint64_t& stop_playback_timestamp,
                               uint64_t& topic_id, uint64_t& timestamp,
                               std::unique_ptr<char[]>& data, size_t& size) {
  if (db_ == nullptr) {
    int ret = OpenNewStorageToPlayback(start_playback_timestamp, stop_playback_timestamp);
    AIMRT_CHECK_WARN(ret, "Open new db failed");
  }

  int ret = sqlite3_step(select_msg_stmt_);
  if (ret == SQLITE_ROW) {
    topic_id = sqlite3_column_int64(select_msg_stmt_, 0);
    timestamp = sqlite3_column_int64(select_msg_stmt_, 1);
    size = sqlite3_column_bytes(select_msg_stmt_, 2);
    const void* buf = sqlite3_column_blob(select_msg_stmt_, 2);
    data = std::make_unique<char[]>(size);
    std::memcpy(data.get(), buf, size);
    return true;
  } else if (ret == SQLITE_DONE) {
    AIMRT_INFO("Reached end of current database file, trying next file...");
    int ret = OpenNewStorageToPlayback(start_playback_timestamp, stop_playback_timestamp);
    ReadRecord(start_playback_timestamp, stop_playback_timestamp, topic_id, timestamp, data, size);
  } else {
    AIMRT_WARN("sqlite3_step failed, ret: {}, error info: {}", ret, sqlite3_errmsg(db_));
    return false;
  }

  return true;
}

size_t SqliteStorage::GetFileSize() const {
  if (cur_db_file_path_.empty() || !std::filesystem::exists(cur_db_file_path_)) return 0;
  return std::filesystem::file_size(cur_db_file_path_);
}

bool SqliteStorage::WriteRecord(uint64_t topic_index, uint64_t timestamp, std::shared_ptr<aimrt::util::BufferArrayView> buffer_view_ptr) {
  if (db_ == nullptr) [[unlikely]] {
    // first record
    OpenNewStorageToRecord(timestamp);
  } else if (cur_data_size_ * estimated_overhead_ >= max_bag_size_) [[unlikely]] {
    estimated_overhead_ = std::max(1.0, static_cast<double>(GetFileSize()) / cur_data_size_);
    AIMRT_INFO("estimated_overhead: {}, max_bag_size: {}, cur_data_size: {}", estimated_overhead_, max_bag_size_, cur_data_size_);
    cur_data_size_ = 0;
    OpenNewStorageToRecord(timestamp);
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

  return true;
}

void SqliteStorage::FlushToDisk(){
  if(db_ == nullptr){
    return ;
  }
  sqlite3_exec(db_, "COMMIT", 0, 0, 0);
  if(storage_policy_.journal_mode == "wal"){
    sqlite3_exec(db_, "CHECKPOINT", 0, 0, 0);
    sqlite3_wal_checkpoint(db_, nullptr);
  }
  buf_array_view_cache_.clear();
  buf_cache_.clear();
  sqlite3_exec(db_, "BEGIN", 0, 0, 0);
}

void SqliteStorage::OpenNewStorageToRecord(uint64_t start_timestamp) {
  CloseRecord();

  std::string cur_db_file_name = bag_base_name_ + "_" + std::to_string(cur_db_file_index_) + ".db3";

  cur_db_file_path_ = (real_bag_path_ / cur_db_file_name).string();

  ++cur_db_file_index_;

  // open db
  int ret = sqlite3_open(cur_db_file_path_.c_str(), &db_);
  AIMRT_CHECK_ERROR_THROW(ret == SQLITE_OK,
                          "Sqlite3 open db file failed, path: {}, ret: {}, error info: {}",
                          cur_db_file_path_, ret, sqlite3_errmsg(db_));

  AIMRT_TRACE("Open new db, path: {}", cur_db_file_path_);

  std::string journal_mode_sql = "PRAGMA journal_mode = " + storage_policy_.journal_mode + ";";
  sqlite3_exec(db_, journal_mode_sql.c_str(), 0, 0, 0);

  std::string synchronous_mode_sql = "PRAGMA synchronous = " + storage_policy_.synchronous_mode + ";";
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

  // start transaction
  sqlite3_exec(db_, "BEGIN", 0, 0, 0);

  // update metadatat.yaml
  metadata_.files.emplace_back(
      MetaData::FileMeta{
          .path = cur_db_file_name,
          .start_timestamp = start_timestamp});

  // check and del db file
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

  return;
}

bool SqliteStorage::OpenNewStorageToPlayback(uint64_t start_playback_timestamp, uint64_t stop_playback_timestamp) {
  ClosePlayback();

  if (cur_db_file_index_ >= metadata_.files.size()) [[unlikely]] {
    AIMRT_INFO("cur_db_file_index_: {}, ALL files : {}, there is no more record file", cur_db_file_index_, metadata_.files.size());
    return false;
  }

  const auto& file = metadata_.files[cur_db_file_index_];
  const auto db_file_path = (real_bag_path_ / file.path).string();
  ++cur_db_file_index_;

  uint64_t cur_db_start_timestamp = file.start_timestamp;

  if (stop_playback_timestamp && cur_db_start_timestamp > stop_playback_timestamp) [[unlikely]] {
    AIMRT_INFO("cur_db_start_timestamp: {}, stop_playback_timestamp: {}", cur_db_start_timestamp, stop_playback_timestamp);
    return false;
  }

  int ret = sqlite3_open(db_file_path.c_str(), &db_);
  AIMRT_CHECK_ERROR_THROW(ret == SQLITE_OK,
                          "Sqlite3 open db file failed, path: {}, ret: {}, error info: {}",
                          db_file_path, ret, sqlite3_errmsg(db_));

  std::string sql = "SELECT topic_id, timestamp, data FROM messages";

  std::vector<std::string> condition;

  if (cur_db_start_timestamp < start_playback_timestamp)
    condition.emplace_back("timestamp >= " + std::to_string(start_playback_timestamp));

  if (!select_topic_id_.empty())
    condition.emplace_back("topic_id IN ( " + select_topic_id_ + " )");

  for (size_t ii = 0; ii < condition.size(); ++ii) {
    if (ii == 0)
      sql += " WHERE ";
    else
      sql += " AND ";

    sql += condition[ii];
  }

  ret = sqlite3_prepare_v3(db_, sql.c_str(), sql.size(), 0, &select_msg_stmt_, nullptr);
  AIMRT_INFO("bag path = {} , sql = {}", db_file_path, sql.c_str());

  AIMRT_CHECK_ERROR_THROW(ret == SQLITE_OK,
                          "Sqlite3 prepare failed, sql: {}, ret: {}, error info: {}",
                          sql, ret, sqlite3_errmsg(db_));

  return true;
}

void SqliteStorage::ClosePlayback() {
  if (db_ != nullptr) {
    if (select_msg_stmt_ != nullptr) {
      sqlite3_finalize(select_msg_stmt_);
      select_msg_stmt_ = nullptr;
    }
    sqlite3_close_v2(db_);
    db_ = nullptr;
  }
}

void SqliteStorage::CloseRecord() {
  if (db_ != nullptr) {
    if (insert_msg_stmt_ != nullptr) {
      sqlite3_exec(db_, "COMMIT", 0, 0, 0);
      buf_array_view_cache_.clear();
      buf_cache_.clear();
      sqlite3_finalize(insert_msg_stmt_);
      insert_msg_stmt_ = nullptr;
    }
    sqlite3_close_v2(db_);
    db_ = nullptr;
  }
}

}  // namespace aimrt::plugins::record_playback_plugin