// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <any>
#include <atomic>
#include <cstdio>
#include <functional>
#include <memory>
#include <source_location>
#include <string>
#include <string_view>
#include <vector>

#include "aimrt_module_cpp_interface/channel/channel_handle.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/context/channel_context.h"
#include "aimrt_module_cpp_interface/context/details/concepts.h"
#include "aimrt_module_cpp_interface/context/details/type_support.h"
#include "aimrt_module_cpp_interface/context/res/channel.h"
#include "aimrt_module_cpp_interface/context/res/service.h"
#include "aimrt_module_cpp_interface/context/rpc_context.h"
#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"

#include "util/exception.h"

#include "aimrt_module_cpp_interface/context/details/thread_context.h"

namespace aimrt::context {

class OpPub;
class OpSub;
class OpCli;
class OpSrv;
class OpLog;

class Context : public std::enable_shared_from_this<Context> {
 public:
  Context() noexcept
      : id_(++global_unique_id_) {
    LetMe();
  }

  explicit Context(aimrt::CoreRef core) noexcept
      : core_(core), id_(++global_unique_id_) {}

  ~Context() = default;

  Context(const Context&) = delete;
  Context& operator=(const Context&) = delete;

  aimrt::CoreRef GetRawRef() const noexcept { return core_; }
  static aimrt::CoreRef GetRawRef(const Context& ctx) noexcept { return ctx.core_; }

  [[nodiscard]] aimrt::logger::LoggerRef GetLogger() const { return core_.GetLogger(); }
  static aimrt::logger::LoggerRef GetLogger(const Context& ctx) { return ctx.core_.GetLogger(); }

  std::string_view GetConfigFilePath() const { return core_.GetConfigurator().GetConfigFilePath(); }

  void LetMe() {
    details::g_thread_ctx = {weak_from_this()};
  }

  void StopRunning() noexcept { is_running_ = false; }

  void SetPubState(ChannelState state) noexcept { pub_state_ = state; }
  ChannelState GetPubState() const noexcept { return pub_state_; }

  void SetSubState(ChannelState state) noexcept { sub_state_ = state; }
  ChannelState GetSubState() const noexcept { return sub_state_; }

  void SetCliState(RpcState state) noexcept { cli_state_ = state; }
  RpcState GetCliState() const noexcept { return cli_state_; }

  void SetSrvState(RpcState state) noexcept { srv_state_ = state; }
  RpcState GetSrvState() const noexcept { return srv_state_; }

  [[nodiscard]] bool Running() const noexcept {
    return is_running_;
  }

  [[nodiscard]] OpPub pub(std::source_location loc = std::source_location::current());
  [[nodiscard]] OpSub sub(std::source_location loc = std::source_location::current());
  [[nodiscard]] OpCli cli(std::source_location loc = std::source_location::current());
  [[nodiscard]] OpSrv srv(std::source_location loc = std::source_location::current());
  [[nodiscard]] OpLog log(std::source_location loc = std::source_location::current());

  [[nodiscard]] aimrt::executor::ExecutorRef CreateExecutor(std::string_view name, std::source_location loc = std::source_location::current()) const {
    AIMRT_ASSERT_WITH_LOC(loc, core_, "Core reference is null when get executor [{}].", name);
    auto ex = core_.GetExecutorManager().GetExecutor(name);
    AIMRT_ASSERT_WITH_LOC(loc, ex, "Get executor [{}] failed.", name);
    return ex;
  }

  // CreatePublisher helpers
  template <class T>
  [[nodiscard]] res::Publisher<T> CreatePublisher(std::string_view topic_name, std::source_location loc = std::source_location::current());

  // CreateSubscriber helpers
  template <class T>
  [[nodiscard]] res::Subscriber<T> CreateSubscriber(std::string_view topic_name, std::source_location loc = std::source_location::current());

  template <class T, concepts::SupportedSubscriber<T> TCallback>
  [[nodiscard]] res::Subscriber<T> CreateSubscriber(std::string_view topic_name, TCallback&& callback, std::source_location loc = std::source_location::current());

  template <class T, concepts::SupportedSubscriber<T> TCallback>
  [[nodiscard]] res::Subscriber<T> CreateSubscriber(std::string_view topic_name, const aimrt::executor::ExecutorRef& exe, TCallback&& callback, std::source_location loc = std::source_location::current());

  // CreateClient for service client class
  template <class TServiceClient>
  [[nodiscard]] TServiceClient CreateClient(std::string_view service_name = "", std::source_location loc = std::source_location::current());

  // CreateServer for service server class
  template <class TServiceServer>
  [[nodiscard]] std::shared_ptr<TServiceServer> CreateServer(std::string_view service_name = "", std::source_location loc = std::source_location::current());

  // 从 RPC 资源中提取客户端调用函数
  template <class Q, class P>
  std::function<aimrt::co::Task<aimrt::rpc::Status>(aimrt::rpc::ContextRef, const Q&, P&)>
  GetClientFunction(const res::Service<Q, P>& srv, std::source_location loc = std::source_location::current());

 private:
  friend class OpPub;
  friend class OpSub;
  friend class OpCli;
  friend class OpSrv;

  template <class T>
  ChannelResource& GetChannelResource(const res::Channel<T>& res, std::source_location loc);

  template <class Q, class P>
  RpcResource& GetRpcResource(const res::Service<Q, P>& res, std::source_location loc);

  template <class T>
  using PublishFunction = std::function<void(aimrt::channel::PublisherRef, aimrt::channel::ContextRef, const T&)>;

  template <class T>
  using ChannelCallback = std::function<void(aimrt::channel::ContextRef, std::shared_ptr<const T>)>;

  template <class T>
  using SubscribeFunction = std::function<bool(
      aimrt::channel::SubscriberRef,
      ChannelCallback<T>,
      aimrt::executor::ExecutorRef)>;

  template <class Q, class P>
  using Client = std::function<aimrt::co::Task<aimrt::rpc::Status>(aimrt::rpc::ContextRef, const Q&, P&)>;

  template <class Q, class P>
  using Server = std::function<aimrt::co::Task<aimrt::rpc::Status>(aimrt::rpc::ContextRef, const Q&, P&)>;

  template <class Q, class P>
  using ServerInvoker = std::function<
      aimrt::co::Task<aimrt::rpc::Status>(aimrt::rpc::ContextRef, const Q&, P&, std::any&)>;

 private:
  aimrt::CoreRef core_;
  std::atomic_bool is_running_{true};
  std::atomic_bool enable_trace_{false};
  std::atomic<ChannelState> pub_state_{ChannelState::kOn};
  std::atomic<ChannelState> sub_state_{ChannelState::kOn};
  std::atomic<RpcState> cli_state_{RpcState::kOn};
  std::atomic<RpcState> srv_state_{RpcState::kOn};
  aimrt::util::DynamicLatch dynamic_latch_;
  int id_{0};

  std::vector<ChannelResource> channel_resources_;
  std::vector<RpcResource> rpc_resources_;

  inline static std::atomic_int global_unique_id_{0};
};

template <class T>
auto Context::GetChannelResource(const res::Channel<T>& res, std::source_location loc) -> ChannelResource& {
  AIMRT_ASSERT_WITH_LOC(loc, res.IsValid(), "Channel [{}] is invalid.", res.GetName());
  AIMRT_ASSERT_WITH_LOC(loc, res.context_id_ == id_, "Channel [{}] belongs to context [{}], but current context is [{}].", res.GetName(), res.context_id_, id_);
  AIMRT_ASSERT_WITH_LOC(loc, res.idx_ < channel_resources_.size(), "Channel [{}] index [{}] out of range (size = {}).", res.GetName(), res.idx_, channel_resources_.size());
  return channel_resources_[res.idx_];
}

template <class Q, class P>
auto Context::GetRpcResource(const res::Service<Q, P>& res, std::source_location loc) -> RpcResource& {
  AIMRT_ASSERT_WITH_LOC(loc, res.IsValid(), "Service [{}] is invalid.", res.GetName());
  AIMRT_ASSERT_WITH_LOC(loc, res.context_id_ == id_, "Service [{}] belongs to context [{}], but current context is [{}].", res.GetName(), res.context_id_, id_);
  AIMRT_ASSERT_WITH_LOC(loc, res.idx_ < rpc_resources_.size(), "Service [{}] index [{}] out of range (size = {}).", res.GetName(), res.idx_, rpc_resources_.size());
  return rpc_resources_[res.idx_];
}

}  // namespace aimrt::context

namespace aimrt::context {

inline bool Running(std::source_location loc = std::source_location::current()) {
  return details::GetCurrentContext(loc)->Running();
}

}  // namespace aimrt::context

#include "aimrt_module_cpp_interface/context/op_cli.h"
#include "aimrt_module_cpp_interface/context/op_log.h"
#include "aimrt_module_cpp_interface/context/op_pub.h"
#include "aimrt_module_cpp_interface/context/op_srv.h"
#include "aimrt_module_cpp_interface/context/op_sub.h"

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

inline OpLog Context::log(std::source_location loc) {
  return OpLog(*this, loc);
}

// Context CreatePublisher helper impl
template <class T>
inline res::Publisher<T> Context::CreatePublisher(std::string_view topic_name, std::source_location loc) {
  return pub(loc).Init<T>(topic_name);
}

// publisher init
template <class T>
std::pair<res::Publisher<T>, ChannelResource&> OpPub::DoInit(std::string_view topic_name) {
  ChannelResource channel_ctx;
  channel_ctx.pub = ctx_.core_.GetChannelHandle().GetPublisher(topic_name);
  AIMRT_ASSERT_WITH_LOC(loc_, channel_ctx.pub, "Get publisher for topic [{}] failed.", topic_name);

  const bool registered = channel_ctx.pub.RegisterPublishType(aimrt::GetMessageTypeSupport<T>());
  AIMRT_ASSERT_WITH_LOC(loc_, registered, "Register publish type for topic [{}] failed.", topic_name);

  ctx_.channel_resources_.push_back(std::move(channel_ctx));

  res::Publisher<T> res;
  res.name_ = std::string(topic_name);
  res.idx_ = ctx_.channel_resources_.size() - 1;
  res.context_id_ = ctx_.id_;
  return {res, ctx_.channel_resources_.back()};
}

// publisher publish
template <class T>
void OpPub::Publish(const res::Channel<T>& ch, aimrt::channel::ContextRef ch_ctx, const T& msg) {
  auto& channel_ctx = ctx_.GetChannelResource(ch, loc_);
  if (!ctx_.Running() || ctx_.GetPubState() != context::ChannelState::kOn) [[unlikely]] {
    ctx_.log().Trace("Publisher is not ready.");
    return;
  }
  std::any_cast<typename Context::PublishFunction<T>&>(channel_ctx.pub_f)(channel_ctx.pub, ch_ctx, msg);
}

template <class T>
void OpPub::Publish(const res::Channel<T>& ch, const T& msg) {
  aimrt::channel::Context ctx;
  Publish(ch, ctx, msg);
}

template <class T>
Context::PublishFunction<T> OpPub::CreatePublishFunction() {
  return [](aimrt::channel::PublisherRef publisher, aimrt::channel::ContextRef ctx_ref, const T& msg) {
    aimrt::channel::PublishMsg(publisher, ctx_ref, msg);
  };
}

// subscriber init
template <class T>
std::pair<res::Subscriber<T>, ChannelResource&> OpSub::DoInit(std::string_view topic_name) {
  ChannelResource channel_ctx;
  channel_ctx.sub = ctx_.core_.GetChannelHandle().GetSubscriber(topic_name);
  AIMRT_ASSERT_WITH_LOC(loc_, channel_ctx.sub, "Get subscriber for topic [{}] failed.", topic_name);

  ctx_.channel_resources_.push_back(std::move(channel_ctx));

  res::Subscriber<T> res;
  res.name_ = std::string(topic_name);
  res.idx_ = ctx_.channel_resources_.size() - 1;
  res.context_id_ = ctx_.id_;
  return {res, ctx_.channel_resources_.back()};
}

// Context CreateSubscriber helpers impls
template <class T>
inline res::Subscriber<T> Context::CreateSubscriber(std::string_view topic_name, std::source_location loc) {
  return sub(loc).Init<T>(topic_name);
}

template <class T, concepts::SupportedSubscriber<T> TCallback>
inline res::Subscriber<T> Context::CreateSubscriber(std::string_view topic_name, TCallback&& callback, std::source_location loc) {
  auto s = sub(loc).Init<T>(topic_name);
  sub(loc).SubscribeInline(s, std::forward<TCallback>(callback));
  return s;
}

template <class T, concepts::SupportedSubscriber<T> TCallback>
inline res::Subscriber<T> Context::CreateSubscriber(std::string_view topic_name, const aimrt::executor::ExecutorRef& exe, TCallback&& callback, std::source_location loc) {
  auto s = sub(loc).Init<T>(topic_name);
  sub(loc).SubscribeOn(s, exe, std::forward<TCallback>(callback));
  return s;
}

// subscriber subscribe inline
template <class T, concepts::SupportedSubscriber<T> TCallback>
void OpSub::SubscribeInline(const res::Subscriber<T>& ch, TCallback callback) {
  DoSubscribe(ch, std::move(callback), {});
}

// subscriber subscribe on
template <class T, concepts::SupportedSubscriber<T> TCallback>
void OpSub::SubscribeOn(const res::Subscriber<T>& ch, aimrt::executor::ExecutorRef exe, TCallback callback) {
  DoSubscribe(ch, std::move(callback), std::move(exe));
}

// subscriber subscribe
template <class T, concepts::SupportedSubscriber<T> TCallback>
void OpSub::DoSubscribe(const res::Subscriber<T>& ch, TCallback callback, aimrt::executor::ExecutorRef exe) {
  auto& channel_ctx = ctx_.GetChannelResource(ch, loc_);
  auto& sub_fn = std::any_cast<typename Context::SubscribeFunction<T>&>(channel_ctx.sub_f);
  auto ctx = details::GetCurrentContext();
  const bool ok = sub_fn(channel_ctx.sub,
                         StandardizeSubscriber<T>(std::move(callback)),
                         std::move(exe));
  AIMRT_ASSERT_WITH_LOC(loc_, ok, "Subscribe [{}] failed.", ch.GetName());
}

// subscriber subscribe function
template <class T>
Context::SubscribeFunction<T> OpSub::CreateSubscribeFunction() {
  return [](aimrt::channel::SubscriberRef subscriber,
            typename Context::ChannelCallback<T> cb,
            aimrt::executor::ExecutorRef exe) {
    return RawSubscribe<T>(subscriber, std::move(exe), std::move(cb));
  };
}

template <class T, class F>
bool OpSub::RawSubscribe(
    aimrt::channel::SubscriberRef subscriber,
    aimrt::executor::ExecutorRef exe,
    F&& callback) {
  auto type_support = aimrt::GetMessageTypeSupport<T>();

  auto cb_ptr = std::make_shared<typename Context::ChannelCallback<T>>(std::forward<F>(callback));
  auto ctx = details::GetCurrentContext();

  if (!exe) {
    return subscriber.Subscribe(
        type_support,
        [cb_ptr, ctx_ptr = ctx->shared_from_this()](
            const aimrt_channel_context_base_t* chn_ctx_ptr,
            const void* msg_ptr,
            aimrt_function_base_t* release_callback_base) mutable {
          channel::SubscriberReleaseCallback release_callback(release_callback_base);
          ctx_ptr->LetMe();

          if (!ctx_ptr->Running() || ctx_ptr->GetSubState() != context::ChannelState::kOn) [[unlikely]] {
            ctx_ptr->log().Trace("Subscriber is not ready.");
            release_callback();
            return;
          }
          (*cb_ptr)(
              aimrt::channel::ContextRef(chn_ctx_ptr),
              std::shared_ptr<const T>(
                  static_cast<const T*>(msg_ptr),
                  [release_callback{std::move(release_callback)}](const T*) { release_callback(); }));
        });
  }

  return subscriber.Subscribe(
      type_support,
      [ctx_ptr = ctx->shared_from_this(), exeref = std::move(exe), cb_ptr = std::move(cb_ptr)](
          const aimrt_channel_context_base_t* chn_ctx_ptr,
          const void* msg_ptr,
          aimrt_function_base_t* release_callback_base) mutable {
        if (!ctx_ptr->Running() || ctx_ptr->GetSubState() != context::ChannelState::kOn) [[unlikely]] {
          ctx_ptr->log().Trace("Subscriber is not ready.");
          return;
        }

        channel::SubscriberReleaseCallback release_callback(release_callback_base);
        auto msg = std::shared_ptr<const T>(
            static_cast<const T*>(msg_ptr),
            [release_callback{std::move(release_callback)}](const T*) { release_callback(); });
        exeref.Execute([cb_ptr, ctx_ptr, chn_ctx_ptr, msg = std::move(msg)]() mutable {
          ctx_ptr->LetMe();
          if (!ctx_ptr->Running() || ctx_ptr->GetSubState() != context::ChannelState::kOn) [[unlikely]] {
            ctx_ptr->log().Trace("Subscriber is not ready.");
            return;
          }
          (*cb_ptr)(aimrt::channel::ContextRef(chn_ctx_ptr), std::move(msg));
        });
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

// Log
template <class... Args>
inline void OpLog::Log(std::uint32_t level, fmt::format_string<Args...> fmt, Args&&... args) {
  const auto& cur_lgr = ctx_.GetLogger();
  if (level >= cur_lgr.GetLogLevel()) {
    std::string log_str = ::aimrt_fmt::format(fmt, std::forward<Args>(args)...);
    cur_lgr.Log(level, loc_.line(), loc_.file_name(), __FUNCTION__, log_str.c_str(), log_str.size());
  }
}

inline OpLog Log(std::source_location loc = std::source_location::current()) {
  return aimrt::context::details::GetCurrentContext(loc)->log(loc);
}

// Context CreateClient for service client class
template <class TServiceClient>
inline TServiceClient Context::CreateClient(std::string_view service_name, std::source_location loc) {
  TServiceClient client;
  client.Init(shared_from_this(), std::string(service_name));
  return client;
}

template <class Q, class P, concepts::RawClient<Q, P> TClient>
res::Client<Q, P> OpCli::Init(std::string_view service_name, TClient&& client) {
  RpcResource rpc_ctx;

  rpc_ctx.call_f = Context::Client<Q, P>(
      [client = std::move(client)](aimrt::rpc::ContextRef ctx, const Q& q, P& p) -> aimrt::co::Task<aimrt::rpc::Status> {
        co_return co_await client(ctx, q, p);
      });

  rpc_ctx.func_name = std::string(service_name);

  ctx_.rpc_resources_.push_back(std::move(rpc_ctx));

  res::Client<Q, P> res;
  res.name_ = std::string(service_name);
  res.idx_ = ctx_.rpc_resources_.size() - 1;
  res.context_id_ = ctx_.id_;

  return res;
}

template <class Q, class P>
co::Task<aimrt::rpc::Status> OpCli::Call(const res::Service<Q, P>& srv, const Q& q, P& p) {
  return Call(srv, aimrt::rpc::ContextRef(), q, p);
}

template <class Q, class P>
co::Task<aimrt::rpc::Status> OpCli::Call(const res::Service<Q, P>& srv, aimrt::rpc::ContextRef ctx, const Q& q, P& p) {
  details::GetCurrentContext(loc_)->LetMe();
  auto& rpc_ctx = ctx_.GetRpcResource(srv, loc_);
  co_return co_await std::any_cast<std::function<aimrt::co::Task<aimrt::rpc::Status>(aimrt::rpc::ContextRef, const Q&, P&)>&>(rpc_ctx.call_f)(ctx, q, p);
}

template <class TServiceServer>
std::shared_ptr<TServiceServer> Context::CreateServer(std::string_view service_name, std::source_location loc) {
  auto server = std::make_shared<TServiceServer>();
  server->Init(shared_from_this(), std::string(service_name));
  return server;
}

// OpSrv::Init implementation
template <class Q, class P>
res::Service<Q, P> OpSrv::Init(std::string_view service_name) {
  RpcResource rpc_ctx;
  rpc_ctx.func_name = std::string(service_name);

  ctx_.rpc_resources_.push_back(std::move(rpc_ctx));

  res::Service<Q, P> res;
  res.name_ = std::string(service_name);
  res.idx_ = ctx_.rpc_resources_.size() - 1;
  res.context_id_ = ctx_.id_;

  return res;
}

// OpSrv::ServeInline implementation
template <class Q, class P, concepts::SupportedServer<Q, P> TServer>
void OpSrv::ServeInline(const res::Service<Q, P>& srv, TServer server) {
  auto& rpc_ctx = ctx_.GetRpcResource(srv, loc_);

  auto standardized_server = StandardizeServer<Q, P, TServer>(std::move(server));
  rpc_ctx.serve_f = Context::Server<Q, P>(
      [server_func = std::move(standardized_server), ctx_ptr = ctx_.weak_from_this()](
          aimrt::rpc::ContextRef rpc_ctx, const Q& q, P& p) -> aimrt::co::Task<aimrt::rpc::Status> {
        if (auto ctx = ctx_ptr.lock()) {
          ctx->LetMe();
          co_return co_await server_func(rpc_ctx, q, p);
        }
        co_return aimrt::rpc::Status(AIMRT_RPC_STATUS_SVR_NOT_READY);
      });
}

// OpSrv::ServeOn implementation
template <class Q, class P, concepts::SupportedServer<Q, P> TServer>
void OpSrv::ServeOn(const res::Service<Q, P>& srv, aimrt::executor::ExecutorRef exe, TServer server) {
  auto& rpc_ctx = ctx_.GetRpcResource(srv, loc_);

  auto standardized_server = StandardizeServer<Q, P, TServer>(std::move(server));
  auto ctx_ptr = ctx_.shared_from_this();

  rpc_ctx.serve_f = Context::Server<Q, P>(
      [server_func = std::move(standardized_server), exe = std::move(exe), ctx_ptr](
          aimrt::rpc::ContextRef ctx, const Q& q, P& p) -> aimrt::co::Task<aimrt::rpc::Status> {
        co_await aimrt::co::Schedule(aimrt::co::AimRTScheduler(exe));
        ctx_ptr->LetMe();
        co_return co_await server_func(ctx, q, p);
      });
}

// OpSrv::Serving implementation
template <class Q, class P>
aimrt::co::Task<aimrt::rpc::Status> OpSrv::Serving(const res::Service<Q, P>& srv, aimrt::rpc::ContextRef ctx, const Q& q, P& p) {
  details::GetCurrentContext(loc_)->LetMe();
  auto& rpc_ctx = ctx_.GetRpcResource(srv, loc_);

  if (!rpc_ctx.serve_f.has_value()) {
    co_return aimrt::rpc::Status(AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED);
  }

  if (ctx_.Running() || ctx_.GetSrvState() != context::RpcState::kOn) [[unlikely]] {
    ctx_.log().Trace("Server is not ready to serve.");
    co_return aimrt::rpc::Status(AIMRT_RPC_STATUS_SVR_NOT_READY);
  }

  co_return co_await std::any_cast<Context::Server<Q, P>&>(rpc_ctx.serve_f)(ctx, q, p);
}

// OpSrv::StandardizeServer implementation
template <class Q, class P, concepts::SupportedServer<Q, P> Func>
constexpr auto OpSrv::StandardizeServer(Func&& cb) {
  using namespace concepts;
  using ServerFunc = std::function<aimrt::co::Task<aimrt::rpc::Status>(aimrt::rpc::ContextRef, const Q&, P&)>;

  // Handle coroutine types (already return Task<Status>)
  if constexpr (ServerCoroutineWithCtx<decltype(cb), Q, P>) {
    return ServerFunc(std::forward<Func>(cb));
  } else if constexpr (ServerCoroutine<decltype(cb), Q, P>) {
    return ServerFunc([callback = std::forward<Func>(cb)](
                          aimrt::rpc::ContextRef, const Q& q, P& p) -> aimrt::co::Task<aimrt::rpc::Status> {
      return callback(q, p);
    });
  } else if constexpr (ServerCoroutineReturnVoidWithCtx<decltype(cb), Q, P>) {
    return ServerFunc([callback = std::forward<Func>(cb)](
                          aimrt::rpc::ContextRef ctx, const Q& q, P& p) -> aimrt::co::Task<aimrt::rpc::Status> {
      co_await callback(ctx, q, p);
      co_return aimrt::rpc::Status();
    });
  } else if constexpr (ServerCoroutineReturnVoid<decltype(cb), Q, P>) {
    return ServerFunc([callback = std::forward<Func>(cb)](
                          aimrt::rpc::ContextRef, const Q& q, P& p) -> aimrt::co::Task<aimrt::rpc::Status> {
      co_await callback(q, p);
      co_return aimrt::rpc::Status();
    });
  }
  // Handle synchronous function types (wrap in coroutine)
  else if constexpr (ServerFunctionWithCtx<decltype(cb), Q, P>) {
    return ServerFunc([callback = std::forward<Func>(cb)](
                          aimrt::rpc::ContextRef ctx, const Q& q, P& p) -> aimrt::co::Task<aimrt::rpc::Status> {
      co_return callback(ctx, q, p);
    });
  } else if constexpr (ServerFunctionReturnVoidWithCtx<decltype(cb), Q, P>) {
    return ServerFunc([callback = std::forward<Func>(cb)](
                          aimrt::rpc::ContextRef ctx, const Q& q, P& p) -> aimrt::co::Task<aimrt::rpc::Status> {
      callback(ctx, q, p);
      co_return aimrt::rpc::Status();
    });
  } else if constexpr (ServerFunction<decltype(cb), Q, P>) {
    return ServerFunc([callback = std::forward<Func>(cb)](
                          aimrt::rpc::ContextRef, const Q& q, P& p) -> aimrt::co::Task<aimrt::rpc::Status> {
      co_return callback(q, p);
    });
  } else if constexpr (ServerFunctionReturnVoid<decltype(cb), Q, P>) {
    return ServerFunc([callback = std::forward<Func>(cb)](
                          aimrt::rpc::ContextRef, const Q& q, P& p) -> aimrt::co::Task<aimrt::rpc::Status> {
      callback(q, p);
      co_return aimrt::rpc::Status();
    });
  } else {
    static_assert(sizeof(decltype(cb)) == 0, "Unsupported server callback type.");
  }
}

}  // namespace aimrt::context

namespace aimrt::context::res {

template <class T>
inline void Publisher<T>::Publish(const T& msg, std::source_location loc) const {
  aimrt::context::details::GetCurrentContext(loc)->pub().Publish(*this, msg);
}

template <class T>
inline void Publisher<T>::Publish(aimrt::channel::ContextRef ch_ctx, const T& msg, std::source_location loc) const {
  aimrt::context::details::GetCurrentContext(loc)->pub().Publish(*this, ch_ctx, msg);
}

template <class T>
template <concepts::SupportedSubscriber<T> TCallback>
inline Subscriber<T> Subscriber<T>::SubscribeOn(const aimrt::executor::ExecutorRef& exe, TCallback callback, std::source_location loc) const {
  aimrt::context::details::GetCurrentContext(loc)->sub().SubscribeOn(*this, exe, std::move(callback));
  return *this;
}

template <class T>
template <concepts::SupportedSubscriber<T> TCallback>
inline Subscriber<T> Subscriber<T>::SubscribeInline(TCallback callback, std::source_location loc) const {
  aimrt::context::details::GetCurrentContext(loc)->sub().SubscribeInline(*this, std::move(callback));
  return *this;
}

// Implementation of Client::Call method
template <class Q, class P>
inline co::Task<aimrt::rpc::Status> Client<Q, P>::Call(
    const Q& q, P& p, std::source_location loc) const {
  return Call(aimrt::rpc::ContextRef(), q, p, loc);
}

template <class Q, class P>
inline co::Task<aimrt::rpc::Status> Client<Q, P>::Call(
    aimrt::rpc::ContextRef ctx, const Q& q, P& p, std::source_location loc) const {
  auto ctx_ptr = aimrt::context::details::GetCurrentContext(loc);
  if (ctx_ptr->GetCliState() != context::RpcState::kOn) [[unlikely]] {
    co_return aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_NOT_READY);
  }
  co_return co_await ctx_ptr->cli().Call(*this, ctx, q, p);
}

// Implementation of Server methods
template <class Q, class P>
template <typename TServer>
inline void Server<Q, P>::ServeInline(TServer server, std::source_location loc) const {
  auto ctx_ptr = aimrt::context::details::GetCurrentContext(loc);
  ctx_ptr->srv(loc).ServeInline(*this, std::move(server));
}

template <class Q, class P>
template <typename TServer>
inline void Server<Q, P>::ServeOn(
    const aimrt::executor::ExecutorRef& exe, TServer server, std::source_location loc) const {
  auto ctx_ptr = aimrt::context::details::GetCurrentContext(loc);
  ctx_ptr->srv(loc).ServeOn(*this, exe, std::move(server));
}

}  // namespace aimrt::context::res

#ifdef AIMRT_CONTEXT_DEFINE_GLOBAL_LOGGER_HANDLE

inline aimrt::logger::LoggerRef GetLogger() {
  if (!aimrt::context::details::g_thread_ctx.ctx_ptr.expired()) {
    return aimrt::context::details::GetCurrentContext()->GetRawRef().GetLogger();
  }
  return aimrt::logger::GetSimpleLoggerRef();
}

#endif
