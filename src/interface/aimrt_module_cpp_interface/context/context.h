// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <any>
#include <functional>
#include <memory>
#include <source_location>
#include <string>
#include <string_view>
#include <vector>

#include "aimrt_module_cpp_interface/channel/channel_handle.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/context/channel_context.h"
#include "aimrt_module_cpp_interface/context/details/concepts.h"
#include "aimrt_module_cpp_interface/context/details/type_support.h"
#include "aimrt_module_cpp_interface/context/res/channel.h"
#include "aimrt_module_cpp_interface/context/res/executor.h"
#include "aimrt_module_cpp_interface/context/res/service.h"
#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "util/exception.h"

namespace aimrt::context {

class OpPub;
class OpSub;
class OpCli;
class OpSrv;

class Context : public std::enable_shared_from_this<Context> {
 public:
  Context() noexcept
      : id_(++global_unique_id_) {}

  explicit Context(aimrt::CoreRef core) noexcept
      : core_(core), id_(++global_unique_id_) {}

  ~Context()  = default;

  Context(const Context&) = delete;
  Context& operator=(const Context&) = delete;

  [[nodiscard]] aimrt::CoreRef Core() const noexcept { return core_; }
  static aimrt::CoreRef GetRawRef(const Context& ctx) noexcept { return ctx.core_; }

  static aimrt::logger::LoggerRef GetLogger(const Context& ctx) noexcept { return ctx.core_.GetLogger(); }

  void RequireToShutdown() noexcept { is_ok_.store(false, std::memory_order_relaxed); }
  [[nodiscard]] bool Ok() const noexcept { return is_ok_.load(std::memory_order_relaxed); }

  [[nodiscard]] OpPub pub(std::source_location loc = std::source_location::current());
  [[nodiscard]] OpSub sub(std::source_location loc = std::source_location::current());
  [[nodiscard]] OpCli cli(std::source_location loc = std::source_location::current());
  [[nodiscard]] OpSrv srv(std::source_location loc = std::source_location::current());

  [[nodiscard]] aimrt::executor::ExecutorRef GetExecutor(std::string_view name) const {
    AIMRT_ASSERT(core_, "Core reference is null when get executor [{}].", name);
    auto ex = core_.GetExecutorManager().GetExecutor(name);
    AIMRT_ASSERT(ex, "Get executor [{}] failed.", name);
    return ex;
  }

 private:
  friend class OpPub;
  friend class OpSub;
  friend class OpCli;
  friend class OpSrv;

  template <class T>
  ChannelContext& GetChannelContext(const res::Channel<T>& res, std::source_location loc);

  struct ServiceContext {
    std::string func_name;
    std::any call_f;
    std::any serve_f;
  };

  template <class Q, class P>
  ServiceContext& GetServiceContext(const res::Service<Q, P>& res, std::source_location loc);

  aimrt::CoreRef core_;
  std::atomic_bool is_ok_{true};
  std::atomic_bool enable_trace_{false};
  int id_{0};

  std::vector<ChannelContext> channel_contexts_;
  std::vector<ServiceContext> service_contexts_;

  inline static std::atomic_int global_unique_id_{0};

  template <class T>
  using PublishFunction = std::function<void(aimrt::channel::PublisherRef, aimrt::channel::ContextRef, const T&)>;

  template <class T>
  using ChannelCallback = std::function<void(aimrt::channel::ContextRef, std::shared_ptr<const T>)>;

  template <class T>
  using SubscribeFunction = std::function<bool(
      aimrt::channel::SubscriberRef,
      ChannelCallback<T>,
      std::weak_ptr<Context>,
      aimrt::executor::ExecutorRef)>;

  template <class Q, class P>
  using Client = std::function<aimrt::co::Task<aimrt::rpc::Status>(aimrt::rpc::ContextRef, const Q&, P&)>;

  template <class Q, class P>
  using Server = std::function<aimrt::co::Task<aimrt::rpc::Status>(aimrt::rpc::ContextRef, const Q&, P&)>;
};


template <class T>
auto Context::GetChannelContext(const res::Channel<T>& res, std::source_location loc) -> ChannelContext& {
  (void)loc;
  AIMRT_ASSERT(res.IsValid(), "Channel [{}] is invalid.", res.GetName());
  AIMRT_ASSERT(
      res.context_id_ == id_,
      "Channel [{}] belongs to context [{}], but current context is [{}].",
      res.GetName(),
      res.context_id_,
      id_);
  AIMRT_ASSERT(
      res.idx_ < channel_contexts_.size(),
      "Channel [{}] index [{}] out of range (size = {}).",
      res.GetName(),
      res.idx_,
      channel_contexts_.size());
  return channel_contexts_[res.idx_];
}

template <class Q, class P>
auto Context::GetServiceContext(const res::Service<Q, P>& res, std::source_location loc) -> ServiceContext& {
  (void)loc;
  AIMRT_ASSERT(res.IsValid(), "Service [{}] is invalid.", res.GetName());
  AIMRT_ASSERT(
      res.context_id_ == id_,
      "Service [{}] belongs to context [{}], but current context is [{}].",
      res.GetName(),
      res.context_id_,
      id_);
  AIMRT_ASSERT(
      res.idx_ < service_contexts_.size(),
      "Service [{}] index [{}] out of range (size = {}).",
      res.GetName(),
      res.idx_,
      service_contexts_.size());
  return service_contexts_[res.idx_];
}

}  // namespace aimrt::context

#include "aimrt_module_cpp_interface/context/op_pub.h"
#include "aimrt_module_cpp_interface/context/op_sub.h"
#include "aimrt_module_cpp_interface/context/op_cli.h"
#include "aimrt_module_cpp_interface/context/op_srv.h"

namespace aimrt::context {

// Context member function implementations
inline OpPub Context::pub(std::source_location loc) {
  return OpPub(*this, loc);
}

inline OpSub Context::sub(std::source_location loc) {
  return OpSub(*this, loc);
}

inline OpCli Context::cli(std::source_location loc) {
  return OpCli(*this, loc);
}

inline OpSrv Context::srv(std::source_location loc) {
  return OpSrv(*this, loc);
}

template <class T>
void OpPub::Publish(const res::Channel<T>& ch, aimrt::channel::ContextRef ch_ctx, const T& msg) {
  auto& channel_ctx = ctx_.GetChannelContext(ch, loc_);

  std::any_cast<typename Context::PublishFunction<T>&>(channel_ctx.pub_f)(channel_ctx.pub, ch_ctx, msg);
}

template <class T>
std::pair<res::Channel<T>, ChannelContext&> OpPub::DoInit(std::string_view topic_name) {
  static_assert(concepts::DirectlySupportedType<T>, "Channel type must be directly supported.");

  ChannelContext channel_ctx;
  channel_ctx.pub = ctx_.core_.GetChannelHandle().GetPublisher(topic_name);
  AIMRT_ASSERT(channel_ctx.pub, "Get publisher for topic [{}] failed.", topic_name);

  const bool registered = details::RegisterPublishType<T>(channel_ctx.pub);
  AIMRT_ASSERT(registered, "Register publish type for topic [{}] failed.", topic_name);

  ctx_.channel_contexts_.push_back(std::move(channel_ctx));

  res::Channel<T> res;
  res.name_ = std::string(topic_name);
  res.idx_ = ctx_.channel_contexts_.size() - 1;
  res.context_id_ = ctx_.id_;
  return {res, ctx_.channel_contexts_.back()};
}

// OpSub member function implementations
template <class T>
std::pair<res::Channel<T>, ChannelContext&> OpSub::DoInit(std::string_view topic_name) {
  static_assert(concepts::DirectlySupportedType<T>, "Channel type must be directly supported.");

  ChannelContext channel_ctx;
  channel_ctx.sub = ctx_.core_.GetChannelHandle().GetSubscriber(topic_name);
  AIMRT_ASSERT(channel_ctx.sub, "Get subscriber for topic [{}] failed.", topic_name);

  ctx_.channel_contexts_.push_back(std::move(channel_ctx));

  res::Channel<T> res;
  res.name_ = std::string(topic_name);
  res.idx_ = ctx_.channel_contexts_.size() - 1;
  res.context_id_ = ctx_.id_;
  return {res, ctx_.channel_contexts_.back()};
}

template <class T, concepts::SupportedSubscriber<T> TCallback>
void OpSub::SubscribeInline(const res::Channel<T>& ch, TCallback callback) {
  DoSubscribe(ch, std::forward<TCallback>(callback), {});
}

template <class T, concepts::SupportedSubscriber<T> TCallback>
void OpSub::SubscribeOn(const res::Channel<T>& ch, aimrt::executor::ExecutorRef exe, TCallback callback) {
  DoSubscribe(ch, std::forward<TCallback>(callback), std::move(exe));
}

template <class T, concepts::SupportedSubscriber<T> TCallback>
void OpSub::DoSubscribe(const res::Channel<T>& ch, TCallback callback, aimrt::executor::ExecutorRef exe) {
  auto& channel_ctx = ctx_.GetChannelContext(ch, loc_);
  auto& sub_fn = std::any_cast<typename Context::SubscribeFunction<T>&>(channel_ctx.sub_f);
  const bool ok = sub_fn(channel_ctx.sub,
                         StandardizeSubscriber<T>(std::forward<TCallback>(callback)),
                         ctx_.weak_from_this(),
                         std::move(exe));
  AIMRT_ASSERT(ok, "Subscribe [{}] failed.", ch.GetName());
}

template <concepts::DirectlySupportedType T>
typename Context::SubscribeFunction<T> OpSub::CreateSubscribeFunction() {
  return [](aimrt::channel::SubscriberRef subscriber,
            typename Context::ChannelCallback<T> cb,
            std::weak_ptr<Context> ctx_ptr,
            aimrt::executor::ExecutorRef exe) {
    return RawSubscribe<T>(subscriber, std::move(ctx_ptr), std::move(exe), std::move(cb));
  };
}

template <concepts::DirectlySupportedType T, class F>
bool OpSub::RawSubscribe(
    aimrt::channel::SubscriberRef subscriber,
    std::weak_ptr<Context> ctx_weak,
    aimrt::executor::ExecutorRef exe,
    F&& callback) {
  auto type_support = details::GetMessageTypeSupport<T>();

  auto cb_ptr = std::make_shared<typename Context::ChannelCallback<T>>(std::forward<F>(callback));

  if (!exe) {
    return subscriber.Subscribe(
        type_support,
        [cb_ptr](
            const aimrt_channel_context_base_t* ctx_ptr,
            const void* msg_ptr,
            aimrt_function_base_t* release_callback_base) mutable {
            channel::SubscriberReleaseCallback release_callback(release_callback_base);
            (*cb_ptr)(
                aimrt::channel::ContextRef(ctx_ptr),
                std::shared_ptr<const T>(
                    static_cast<const T*>(msg_ptr),
                    [release_callback{std::move(release_callback)}](const T*) { release_callback(); }
                )
            );
        });
  }

  return subscriber.Subscribe(
      type_support,
      [ctx_weak = std::move(ctx_weak), exe = std::move(exe), cb_ptr = std::move(cb_ptr)](
          const aimrt_channel_context_base_t* ctx_ptr,
          const void* msg_ptr,
          aimrt_function_base_t* release_callback_base) mutable {
        channel::SubscriberReleaseCallback release_callback(release_callback_base);
        if (auto ctx = ctx_weak.lock()) {
          auto msg = std::shared_ptr<const T>(
              static_cast<const T*>(msg_ptr),
              [release_callback{std::move(release_callback)}](const T*) { release_callback(); }
          );
          auto callback_copy = *cb_ptr;
          exe.Execute([cb = std::move(callback_copy), ctx, ctx_ptr, msg = std::move(msg)]() mutable {
            cb(aimrt::channel::ContextRef(ctx_ptr), std::move(msg));
          });
        }
      });
}

template <class T, concepts::SupportedSubscriber<T> F>
auto OpSub::StandardizeSubscriber(F&& cb) {
  using Callback = typename Context::ChannelCallback<T>;

  if constexpr (concepts::SubscriberFunctionWithCtx<F, T>) {
    return Callback(std::forward<F>(cb));
  } else if constexpr (concepts::SubscriberFunctionDerefWithCtx<F, T>) {
    return Callback([callback = std::forward<F>(cb)](aimrt::channel::ContextRef ctx, std::shared_ptr<const T> msg) {
      callback(ctx, *msg);
    });
  } else if constexpr (concepts::SubscriberFunction<F, T>) {
    return Callback([callback = std::forward<F>(cb)](aimrt::channel::ContextRef, std::shared_ptr<const T> msg) {
      callback(std::move(msg));
    });
  } else if constexpr (concepts::SubscriberFunctionDeref<F, T>) {
    return Callback([callback = std::forward<F>(cb)](aimrt::channel::ContextRef, std::shared_ptr<const T> msg) {
      callback(*msg);
    });
  } else {
    static_assert(sizeof(F) == 0, "Unsupported subscriber callback type.");
  }
}

}  // namespace aimrt::context
