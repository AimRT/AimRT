// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/logger/rotate_file_logger_backend.h"
#include <zlib.h>
#include <zstd.h>
#include <charconv>
#include <filesystem>
#include <map>
#include <mutex>
#include <optional>
#include <regex>
#include <vector>
#include "util/exception.h"
#include "util/string_util.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::logger::RotateFileLoggerBackend::Options> {
  using Options = aimrt::runtime::core::logger::RotateFileLoggerBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;
    node["path"] = rhs.path;
    node["filename"] = rhs.filename;
    node["max_file_size_m"] = rhs.max_file_size_m;
    node["max_file_num"] = rhs.max_file_num;
    node["module_filter"] = rhs.module_filter;
    node["pattern"] = rhs.pattern;
    node["enable_sync"] = rhs.enable_sync;
    node["sync_interval_ms"] = rhs.sync_interval_ms;
    node["sync_executor_name"] = rhs.sync_executor_name;
    node["suffix_with_timestamp"] = rhs.suffix_with_timestamp;
    node["compression_mode"] = rhs.compression_mode;
    node["compression_level"] = rhs.compression_level;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["path"]) rhs.path = node["path"].as<std::string>();
    if (node["filename"]) rhs.filename = node["filename"].as<std::string>();
    if (node["max_file_size_m"])
      rhs.max_file_size_m = node["max_file_size_m"].as<uint32_t>();
    if (node["max_file_num"])
      rhs.max_file_num = node["max_file_num"].as<uint32_t>();
    if (node["module_filter"])
      rhs.module_filter = node["module_filter"].as<std::string>();
    if (node["pattern"])
      rhs.pattern = node["pattern"].as<std::string>();
    if (node["sync_interval_ms"])
      rhs.sync_interval_ms = node["sync_interval_ms"].as<uint32_t>();
    if (node["sync_executor_name"])
      rhs.sync_executor_name = node["sync_executor_name"].as<std::string>();
    if (node["enable_sync"])
      rhs.enable_sync = node["enable_sync"].as<bool>();
    if (node["suffix_with_timestamp"])
      rhs.suffix_with_timestamp = node["suffix_with_timestamp"].as<bool>();
    if (node["compression_mode"])
      rhs.compression_mode = node["compression_mode"].as<std::string>();
    if (node["compression_level"])
      rhs.compression_level = node["compression_level"].as<std::string>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::logger {

namespace {

using CompressionMode = RotateFileLoggerBackend::CompressionMode;
using CompressionLevel = RotateFileLoggerBackend::CompressionLevel;

constexpr std::string_view kGzipFileExtension = ".gz";
constexpr std::string_view kZstdFileExtension = ".zst";

constexpr size_t kCompressBufSize = 128 * 1024;

std::string_view GetCompressedFileExtension(CompressionMode mode) {
  return (mode == CompressionMode::kGzip) ? kGzipFileExtension : kZstdFileExtension;
}

// zlib deflate level, valid range [1, 9], 6 is the zlib default level.
// note that gzopen only accepts a digit in its mode string, so
// Z_DEFAULT_COMPRESSION can not be used here
int GetGzipLevel(CompressionLevel level) {
  switch (level) {
    case CompressionLevel::kFast:
      return 1;
    case CompressionLevel::kSlow:
      return 9;
    default:
      return 6;
  }
}

// zstd compression level, valid range [1, 22]
int GetZstdLevel(CompressionLevel level) {
  switch (level) {
    case CompressionLevel::kFast:
      return 1;
    case CompressionLevel::kSlow:
      return 19;
    default:
      return ZSTD_CLEVEL_DEFAULT;
  }
}

// rotated log file is named as '<base>_<index>[_<timestamp>][<compressed file extension>]',
// returns the index, or nullopt if the file does not belong to this backend
std::optional<uint32_t> ParseRotatedFileIndex(
    std::string_view file_name, std::string_view base_file_name) {
  if (file_name.size() <= base_file_name.size() + 1) return std::nullopt;
  if (!file_name.starts_with(base_file_name)) return std::nullopt;
  if (file_name[base_file_name.size()] != '_') return std::nullopt;

  // compressed files keep the rotated name and only append an extension
  for (auto extension : {kGzipFileExtension, kZstdFileExtension}) {
    if (file_name.ends_with(extension)) {
      file_name.remove_suffix(extension.size());
      break;
    }
  }

  std::string_view suffix = file_name.substr(base_file_name.size() + 1);
  if (auto sep = suffix.find('_'); sep != std::string_view::npos) suffix = suffix.substr(0, sep);

  if (!aimrt::common::util::IsDigitStr(suffix)) return std::nullopt;

  uint32_t idx = 0;
  auto result = std::from_chars(suffix.data(), suffix.data() + suffix.size(), idx);
  if (result.ec != std::errc()) return std::nullopt;

  return idx;
}

}  // namespace

RotateFileLoggerBackend::~RotateFileLoggerBackend() {
  if (options_.enable_sync) {
    (void)fflush(file_);
    (void)fclose(file_);
    file_ = nullptr;
    return;
  }
  if (ofs_.is_open()) {
    ofs_.flush();
    ofs_.clear();
    ofs_.close();
  }
}

void RotateFileLoggerBackend::Initialize(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  std::filesystem::path log_path(options_.path);
  base_file_name_ = (log_path / options_.filename).string();

  if (!(std::filesystem::exists(log_path) && std::filesystem::is_directory(log_path))) {
    std::filesystem::create_directories(log_path);
  }

  log_executor_ = get_executor_func_("");  // if input an  empty string , use guard_thread_executor
  AIMRT_ASSERT(log_executor_, "Guard_thread_executor is invalid.");

  if (!options_.pattern.empty()) {
    pattern_ = options_.pattern;
  }
  formatter_.SetPattern(pattern_);

  static const std::map<std::string, CompressionMode, std::less<>> kCompressionModeMap{
      {"none", CompressionMode::kNone},
      {"gzip", CompressionMode::kGzip},
      {"zstd", CompressionMode::kZstd}};
  static const std::map<std::string, CompressionLevel, std::less<>> kCompressionLevelMap{
      {"fast", CompressionLevel::kFast},
      {"default", CompressionLevel::kDefault},
      {"slow", CompressionLevel::kSlow}};

  auto compression_mode_itr = kCompressionModeMap.find(options_.compression_mode);
  AIMRT_ASSERT(compression_mode_itr != kCompressionModeMap.end(),
               "Invalid compression mode: {}, optional values are none/gzip/zstd.",
               options_.compression_mode);
  compression_mode_ = compression_mode_itr->second;

  auto compression_level_itr = kCompressionLevelMap.find(options_.compression_level);
  AIMRT_ASSERT(compression_level_itr != kCompressionLevelMap.end(),
               "Invalid compression level: {}, optional values are fast/default/slow.",
               options_.compression_level);
  compression_level_ = compression_level_itr->second;

  // if enable_sync, set sync timer
  if (options_.enable_sync) {
    // if enable_sync, sync_executor_name must be set
    AIMRT_ASSERT(!options_.sync_executor_name.empty(), "Sync executor name is empty.");

    timer_executor_ = get_executor_func_(options_.sync_executor_name);
    AIMRT_ASSERT(timer_executor_, "Invalid sync executor name: {}", options_.sync_executor_name);
    AIMRT_ASSERT(timer_executor_.SupportTimerSchedule(),
                 "Sync executor {} must support timer schedule.", options_.sync_executor_name);

    // define a timer task to put sync work into log executor
    auto timer_task = [this]() {
      auto sync_work = [this]() {
        (void)fflush(file_);
        if (!logger::Fsync(file_)) {
          (void)fprintf(stderr, "sync log file:  %s failed.\n", base_file_name_.c_str());
        }
      };
      log_executor_.Execute(std::move(sync_work));
    };
    sync_timer_ = executor::CreateTimer(timer_executor_,
                                        std::chrono::milliseconds(options_.sync_interval_ms),
                                        std::move(timer_task));
  }

  options_node = options_;

  run_flag_.store(true);
}

void RotateFileLoggerBackend::Log(const LogDataWrapper& log_data_wrapper) noexcept {
  try {
    if (!run_flag_.load()) [[unlikely]]
      return;

    if (!CheckLog(log_data_wrapper)) [[unlikely]]
      return;

    std::string log_data_str = formatter_.Format(log_data_wrapper);

    if (!options_.enable_sync) {  // disable sync and use C++ API
      auto log_work = [this, log_data_str{std::move(log_data_str)}]() {
        if (!ofs_.is_open() || ofs_.tellp() > options_.max_file_size_m * 1024 * 1024) {
          if (!OpenNewFile()) return;
        }
        ofs_.write(log_data_str.data(), log_data_str.size()) << std::endl;
      };

      log_executor_.Execute(std::move(log_work));

    } else {  // enable sync and use C API
      auto log_work = [this, log_data_str{std::move(log_data_str)}]() {
        if (!file_ || ftell(file_) > options_.max_file_size_m * 1024 * 1024) {
          if (!OpenNewFile()) return;
        }
        (void)fwrite(log_data_str.data(), 1, log_data_str.size(), file_);
        (void)fputc('\n', file_);
        (void)fflush(file_);
      };

      log_executor_.Execute(std::move(log_work));
    }
  } catch (const std::exception& e) {
    (void)fprintf(stderr, "Log get exception: %s\n", e.what());
  }
}

bool RotateFileLoggerBackend::OpenNewFile() {
  bool rename_flag = false;

  if (!options_.enable_sync) {  // disable sync
    // if log file exceed max size, close it
    if (ofs_.is_open()) {
      rename_flag = (ofs_.tellp() > options_.max_file_size_m * 1024 * 1024);
      ofs_.flush();
      ofs_.clear();
      ofs_.close();
    }

    // rename old log file if needed
    if (rename_flag && (std::filesystem::status(base_file_name_).type() == std::filesystem::file_type::regular)) {
      Rename();
    }

    // create and open new log file
    ofs_.open(base_file_name_, std::ios::app);
    if (!ofs_.is_open()) {
      (void)fprintf(stderr, "open log file %s failed.\n", base_file_name_.c_str());
      return false;
    }

  } else {  // ensable sync
    // if log file exceed max size, close it
    if (file_ != NULL) {
      (void)fseek(file_, 0, SEEK_END);
      rename_flag = (ftell(file_) > (options_.max_file_size_m * 1024 * 1024));
      (void)fflush(file_);
      (void)fclose(file_);
      file_ = nullptr;
    }

    // rename old log file if needed
    if (rename_flag && (std::filesystem::status(base_file_name_).type() == std::filesystem::file_type::regular)) {
      Rename();
    }

    // create and open new log file
    file_ = fopen(base_file_name_.c_str(), "a");
    if (file_ == NULL) {
      (void)fprintf(stderr, "open log file %s failed.\n", base_file_name_.c_str());
      return false;
    }
  }

  // make sure number of log files not exceed max_file_num
  CleanLogFile();

  return true;
}

void RotateFileLoggerBackend::Rename() {
  std::string suffix = std::to_string(GetNextIndex());

  if (options_.suffix_with_timestamp) {
    auto tm = aimrt::common::util::GetCurTm();
    char buf[17];  // YYYYMMDD_hhmmss
    snprintf(buf, sizeof(buf), "_%04d%02d%02d_%02d%02d%02d",
             (tm.tm_year + 1900) % 10000u, (tm.tm_mon + 1) % 100u,
             (tm.tm_mday) % 100u, (tm.tm_hour) % 100u,
             (tm.tm_min) % 100u, (tm.tm_sec) % 100u);
    suffix.append(buf, 16);
  }

  std::string rotated_file_name = base_file_name_ + "_" + suffix;
  std::filesystem::rename(base_file_name_, rotated_file_name);

  if (compression_mode_ != CompressionMode::kNone) {
    CompressFile(rotated_file_name);
  }
}

void RotateFileLoggerBackend::CompressFile(const std::string& src_file_path) {
  std::string dst_file_path =
      src_file_path + std::string(GetCompressedFileExtension(compression_mode_));

  bool ret = false;
  try {
    ret = (compression_mode_ == CompressionMode::kGzip)
              ? CompressGzip(src_file_path, dst_file_path)
              : CompressZstd(src_file_path, dst_file_path);
  } catch (const std::exception& e) {
    (void)fprintf(stderr, "Compress log file %s get exception: %s\n",
                  src_file_path.c_str(), e.what());
  }

  std::error_code ec;
  if (!ret) {
    // keep the original log file and drop the incomplete compressed one
    (void)fprintf(stderr, "compress log file %s failed.\n", src_file_path.c_str());
    std::filesystem::remove(dst_file_path, ec);
    return;
  }

  if (!std::filesystem::remove(src_file_path, ec)) {
    (void)fprintf(stderr, "remove log file %s failed: %s\n",
                  src_file_path.c_str(), ec.message().c_str());
  }
}

bool RotateFileLoggerBackend::CompressGzip(
    const std::string& src_file_path, const std::string& dst_file_path) {
  FILE* src_file = fopen(src_file_path.c_str(), "rb");
  if (src_file == nullptr) return false;

  std::string mode = "wb" + std::to_string(GetGzipLevel(compression_level_));
  gzFile dst_file = gzopen(dst_file_path.c_str(), mode.c_str());
  if (dst_file == nullptr) {
    (void)fclose(src_file);
    return false;
  }

  bool ret = true;
  std::vector<char> buf(kCompressBufSize);

  while (true) {
    size_t read_size = fread(buf.data(), 1, buf.size(), src_file);
    if (read_size == 0) {
      ret = (ferror(src_file) == 0);
      break;
    }

    if (gzwrite(dst_file, buf.data(), static_cast<unsigned int>(read_size)) !=
        static_cast<int>(read_size)) {
      ret = false;
      break;
    }
  }

  (void)fclose(src_file);
  if (gzclose(dst_file) != Z_OK) ret = false;

  return ret;
}

bool RotateFileLoggerBackend::CompressZstd(
    const std::string& src_file_path, const std::string& dst_file_path) {
  ZSTD_CCtx* cctx = ZSTD_createCCtx();
  if (cctx == nullptr) return false;

  (void)ZSTD_CCtx_setParameter(cctx, ZSTD_c_compressionLevel, GetZstdLevel(compression_level_));

  FILE* src_file = fopen(src_file_path.c_str(), "rb");
  FILE* dst_file = (src_file == nullptr) ? nullptr : fopen(dst_file_path.c_str(), "wb");
  if (dst_file == nullptr) {
    if (src_file != nullptr) (void)fclose(src_file);
    (void)ZSTD_freeCCtx(cctx);
    return false;
  }

  bool ret = true;
  std::vector<char> in_buf(ZSTD_CStreamInSize());
  std::vector<char> out_buf(ZSTD_CStreamOutSize());

  while (ret) {
    size_t read_size = fread(in_buf.data(), 1, in_buf.size(), src_file);
    if (read_size < in_buf.size() && ferror(src_file) != 0) {
      ret = false;
      break;
    }

    bool last_chunk = (feof(src_file) != 0);
    ZSTD_inBuffer input{in_buf.data(), read_size, 0};

    // one input chunk may need several output chunks to be flushed out
    while (true) {
      ZSTD_outBuffer output{out_buf.data(), out_buf.size(), 0};
      size_t remaining = ZSTD_compressStream2(cctx, &output, &input,
                                              last_chunk ? ZSTD_e_end : ZSTD_e_continue);
      if (ZSTD_isError(remaining) != 0) {
        ret = false;
        break;
      }

      if (fwrite(out_buf.data(), 1, output.pos, dst_file) != output.pos) {
        ret = false;
        break;
      }

      if (last_chunk ? (remaining == 0) : (input.pos == input.size)) break;
    }

    if (last_chunk) break;
  }

  (void)fclose(src_file);
  if (fclose(dst_file) != 0) ret = false;
  (void)ZSTD_freeCCtx(cctx);

  return ret;
}

void RotateFileLoggerBackend::CleanLogFile() {
  if (options_.max_file_num == 0) return;

  std::filesystem::path log_dir = std::filesystem::path(base_file_name_).parent_path();

  std::map<uint32_t, std::string> log_files;

  const std::filesystem::directory_iterator end_itr;
  for (std::filesystem::directory_iterator itr(log_dir); itr != end_itr; ++itr) {
    const std::string& cur_log_file_name = itr->path().string();

    auto cur_idx = ParseRotatedFileIndex(cur_log_file_name, base_file_name_);
    if (!cur_idx) continue;

    log_files.emplace(*cur_idx, cur_log_file_name);
  }

  if (log_files.size() <= options_.max_file_num) return;

  uint32_t del_num = log_files.size() - options_.max_file_num;
  for (auto& itr : log_files) {
    if (del_num == 0) break;
    std::filesystem::remove(itr.second);
    --del_num;
  }
}

uint32_t RotateFileLoggerBackend::GetNextIndex() {
  uint32_t idx = 1;
  std::filesystem::path log_dir =
      std::filesystem::path(base_file_name_).parent_path();

  const std::filesystem::directory_iterator end_itr;
  for (std::filesystem::directory_iterator itr(log_dir); itr != end_itr;
       ++itr) {
    const std::string& cur_log_file_name = itr->path().string();

    auto cur_idx = ParseRotatedFileIndex(cur_log_file_name, base_file_name_);
    if (!cur_idx) continue;

    if (*cur_idx >= idx) idx = *cur_idx + 1;
  }

  return idx;
}

bool RotateFileLoggerBackend::CheckLog(const LogDataWrapper& log_data_wrapper) {
  {
    std::shared_lock lock(module_filter_map_mutex_);
    auto find_itr = module_filter_map_.find(log_data_wrapper.module_name);
    if (find_itr != module_filter_map_.end()) {
      return find_itr->second;
    }
  }

  bool if_log = false;

  try {
    if (std::regex_match(
            log_data_wrapper.module_name.begin(),
            log_data_wrapper.module_name.end(),
            std::regex(options_.module_filter, std::regex::ECMAScript))) {
      if_log = true;
    }
  } catch (const std::exception& e) {
    (void)fprintf(stderr, "Regex get exception, expr: %s, string: %s, exception info: %s\n",
                  options_.module_filter.c_str(), log_data_wrapper.module_name.data(), e.what());
  }

  std::unique_lock lock(module_filter_map_mutex_);
  module_filter_map_.emplace(log_data_wrapper.module_name, if_log);

  return if_log;
}
}  // namespace aimrt::runtime::core::logger
