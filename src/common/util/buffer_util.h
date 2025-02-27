// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <bit>
#include <cinttypes>
#include <cstring>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>

namespace aimrt::common::util {

// Store the uin64_t type as a small terminal in buf
inline void SetBufFromUint64(char *p, uint64_t n) {
  p[0] = (char)(n & 0xFF);
  p[1] = (char)((n >> 8) & 0xFF);
  p[2] = (char)((n >> 16) & 0xFF);
  p[3] = (char)((n >> 24) & 0xFF);
  p[4] = (char)((n >> 32) & 0xFF);
  p[5] = (char)((n >> 40) & 0xFF);
  p[6] = (char)((n >> 48) & 0xFF);
  p[7] = (char)((n >> 56) & 0xFF);
}

// Retrieve the uin64_t type from buf in the form of a small terminal
inline uint64_t GetUint64FromBuf(const char *p) {
  return (uint64_t)((uint8_t)(p[0])) +
         ((uint64_t)((uint8_t)(p[1])) << 8) +
         ((uint64_t)((uint8_t)(p[2])) << 16) +
         ((uint64_t)((uint8_t)(p[3])) << 24) +
         ((uint64_t)((uint8_t)(p[4])) << 32) +
         ((uint64_t)((uint8_t)(p[5])) << 40) +
         ((uint64_t)((uint8_t)(p[6])) << 48) +
         ((uint64_t)((uint8_t)(p[7])) << 56);
}

// Store the uint32_t type as a small terminal in buf
inline void SetBufFromUint32(char *p, uint32_t n) {
  if constexpr (std::endian::native == std::endian::little) {
    memcpy(p, &n, sizeof(n));
  } else {
    p[0] = (char)(n & 0xFF);
    p[1] = (char)((n >> 8) & 0xFF);
    p[2] = (char)((n >> 16) & 0xFF);
    p[3] = (char)((n >> 24) & 0xFF);
  }
}

// Retrieve the uint32_t type from buf in the form of a small terminal
inline uint32_t GetUint32FromBuf(const char *p) {
  if constexpr (std::endian::native == std::endian::little) {
    return *((uint32_t *)p);
  } else {
    return (uint32_t)((uint8_t)(p[0])) +
           ((uint32_t)((uint8_t)(p[1])) << 8) +
           ((uint32_t)((uint8_t)(p[2])) << 16) +
           ((uint32_t)((uint8_t)(p[3])) << 24);
  }
}

// Store the uint16_t type as a small terminal in buf
inline void SetBufFromUint16(char *p, uint16_t n) {
  if constexpr (std::endian::native == std::endian::little) {
    memcpy(p, &n, sizeof(n));
  } else {
    p[0] = (char)(n & 0xFF);
    p[1] = (char)((n >> 8) & 0xFF);
  }
}

// Retrieve the uint16_t type from buf in the form of a small terminal
inline uint16_t GetUint16FromBuf(const char *p) {
  if constexpr (std::endian::native == std::endian::little) {
    return *((uint16_t *)p);
  } else {
    return (uint16_t)((uint8_t)(p[0])) +
           ((uint16_t)((uint8_t)(p[1])) << 8);
  }
}

enum class BufferLenType : size_t {
  kUInt8 = 1,
  kUInt16 = 2,
  kUInt32 = 4,
  kUInt64 = 8
};

class BufferOutOfBoundsException : public std::runtime_error {
 public:
  BufferOutOfBoundsException() : std::runtime_error("Out of bounds") {}
};

class BufferOperator {
 public:
  BufferOperator(char *ptr, size_t len)
      : start_(ptr), end_(start_ + len), cur_(start_) {}
  ~BufferOperator() = default;

  BufferOperator(const BufferOperator &) = delete;
  BufferOperator &operator=(const BufferOperator &) = delete;

  size_t Skip(size_t len) {
    if (cur_ + len > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    cur_ += len;
    return cur_ - start_;
  }

  void JumpTo(size_t pos) {
    if (pos > end_ - start_) [[unlikely]]
      throw BufferOutOfBoundsException();
    cur_ = start_ + pos;
  }

  void SetUint8(uint8_t n) {
    if (cur_ + sizeof(n) > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    *cur_ = n;
    cur_ += sizeof(n);
  }

  void SetUint16(uint16_t n) {
    if (cur_ + sizeof(n) > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    SetBufFromUint16(cur_, n);
    cur_ += sizeof(n);
  }

  void SetUint32(uint32_t n) {
    if (cur_ + sizeof(n) > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    SetBufFromUint32(cur_, n);
    cur_ += sizeof(n);
  }

  void SetUint64(uint64_t n) {
    if (cur_ + sizeof(n) > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    SetBufFromUint64(cur_, n);
    cur_ += sizeof(n);
  }

  void SetBuffer(const char *data, size_t len) {
    if (cur_ + len > end_) [[unlikely]]
      throw BufferOutOfBoundsException();

    memcpy(cur_, data, len);
    cur_ += len;
  }

  void SetBuffer(std::span<const char> buffer) {
    SetBuffer(buffer.data(), buffer.size());
  }

  void SetString(std::string_view s, BufferLenType len_type = BufferLenType::kUInt64) {
    size_t str_len = s.size();

    if (cur_ + static_cast<size_t>(len_type) + str_len > end_) [[unlikely]]
      throw BufferOutOfBoundsException();

    switch (len_type) {
      case BufferLenType::kUInt8:
        if (str_len > std::numeric_limits<uint8_t>::max()) [[unlikely]] {
          throw std::runtime_error("String length out of range.");
        }
        SetUint8(static_cast<uint8_t>(str_len));
        break;
      case BufferLenType::kUInt16:
        if (str_len > std::numeric_limits<uint16_t>::max()) [[unlikely]] {
          throw std::runtime_error("String length out of range.");
        }
        SetUint16(static_cast<uint16_t>(str_len));
        break;
      case BufferLenType::kUInt32:
        if (str_len > std::numeric_limits<uint32_t>::max()) [[unlikely]] {
          throw std::runtime_error("String length out of range.");
        }
        SetUint32(static_cast<uint32_t>(str_len));
        break;
      case BufferLenType::kUInt64:
        if (str_len > std::numeric_limits<uint64_t>::max()) [[unlikely]] {
          throw std::runtime_error("String length out of range.");
        }
        SetUint64(static_cast<uint64_t>(str_len));
        break;
      default:
        throw std::runtime_error("Invalid buffer len type.");
    }

    memcpy(cur_, s.data(), str_len);
    cur_ += str_len;
  }

  size_t GetRemainingSize() const {
    return end_ - cur_;
  }

 private:
  char *start_;
  char *end_;
  char *cur_;
};

class ConstBufferOperator {
 public:
  ConstBufferOperator(const char *ptr, size_t len)
      : start_(ptr), end_(start_ + len), cur_(start_) {}

  ~ConstBufferOperator() = default;

  ConstBufferOperator(const ConstBufferOperator &) = delete;
  ConstBufferOperator &operator=(const ConstBufferOperator &) = delete;

  size_t Skip(size_t len) {
    if (cur_ + len > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    cur_ += len;
    return cur_ - start_;
  }

  void JumpTo(size_t pos) {
    if (pos > end_ - start_) [[unlikely]]
      throw BufferOutOfBoundsException();
    cur_ = start_ + pos;
  }

  uint8_t GetUint8() {
    if (cur_ + sizeof(uint8_t) > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    uint8_t n = *cur_;
    cur_ += sizeof(uint8_t);
    return n;
  }

  uint16_t GetUint16() {
    if (cur_ + sizeof(uint16_t) > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    uint16_t n = GetUint16FromBuf(cur_);
    cur_ += sizeof(uint16_t);
    return n;
  }

  uint32_t GetUint32() {
    if (cur_ + sizeof(uint32_t) > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    uint32_t n = GetUint32FromBuf(cur_);
    cur_ += sizeof(uint32_t);
    return n;
  }

  uint64_t GetUint64() {
    if (cur_ + sizeof(uint64_t) > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    uint64_t n = GetUint64FromBuf(cur_);
    cur_ += sizeof(uint64_t);
    return n;
  }

  std::span<const char> GetBuffer(size_t len) {
    if (cur_ + len > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    std::span<const char> buffer(cur_, cur_ + len);
    cur_ += len;
    return buffer;
  }

  void GetBuffer(char *dst, size_t len) {
    if (cur_ + len > end_) [[unlikely]]
      throw BufferOutOfBoundsException();
    memcpy(dst, cur_, len);
    cur_ += len;
  }

  std::string_view GetString(BufferLenType len_type = BufferLenType::kUInt64) {
    size_t str_len = 0;
    switch (len_type) {
      case BufferLenType::kUInt8:
        str_len = GetUint8();
        break;
      case BufferLenType::kUInt16:
        str_len = GetUint16();
        break;
      case BufferLenType::kUInt32:
        str_len = GetUint32();
        break;
      case BufferLenType::kUInt64:
        str_len = GetUint64();
        break;
      default:
        throw std::runtime_error("Invalid buffer len type.");
    }

    if (cur_ + str_len > end_) [[unlikely]]
      throw BufferOutOfBoundsException();

    std::string_view s(cur_, str_len);
    cur_ += str_len;
    return s;
  }

  size_t GetRemainingSize() const {
    return end_ - cur_;
  }

  std::span<const char> GetRemainingBuffer() {
    return GetBuffer(GetRemainingSize());
  }

 private:
  const char *const start_;
  const char *const end_;
  const char *cur_;
};
}  // namespace aimrt::common::util
