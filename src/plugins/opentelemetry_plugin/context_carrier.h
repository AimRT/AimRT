// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "opentelemetry_plugin/util.h"

#include "opentelemetry/context/propagation/text_map_propagator.h"

namespace aimrt::plugins::opentelemetry_plugin {

template <typename ContetxRefType>
class ContextCarrier : public opentelemetry::context::propagation::TextMapCarrier {
 public:
  explicit ContextCarrier(ContetxRefType ctx_ref)
      : ctx_ref_(ctx_ref) {}
  ContextCarrier() = default;

  virtual opentelemetry::nostd::string_view Get(
      opentelemetry::nostd::string_view key) const noexcept override {
    std::string real_key = std::string(kCtxKeyPrefix) + std::string(key);
    return ToNoStdStringView(ctx_ref_.GetMetaValue(real_key));
  }

  virtual void Set(opentelemetry::nostd::string_view key,
                   opentelemetry::nostd::string_view value) noexcept override {
    std::string real_key = std::string(kCtxKeyPrefix) + std::string(key);
    ctx_ref_.SetMetaValue(real_key, ToStdStringView(value));
  }

  ContetxRefType ctx_ref_;
};
}  // namespace aimrt::plugins::opentelemetry_plugin