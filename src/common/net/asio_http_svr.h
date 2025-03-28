// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <functional>
#include <list>
#include <memory>
#include <utility>

#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include "net/http_dispatcher.h"
#include "util/log_util.h"
#include "util/string_util.h"
#include "util/url_parser.h"

namespace aimrt::common::net {

class AsioHttpServer : public std::enable_shared_from_this<AsioHttpServer> {
 public:
  using IOCtx = boost::asio::io_context;
  using Tcp = boost::asio::ip::tcp;
  using Strand = boost::asio::strand<IOCtx::executor_type>;
  using Timer = boost::asio::steady_timer;

  template <class T>
  using Awaitable = boost::asio::awaitable<T>;

  template <class Body>
  using Request = boost::beast::http::request<Body>;

  template <class Body>
  using Response = boost::beast::http::response<Body>;

  using ReqBodyType = boost::beast::http::dynamic_body;

  enum class HttpHandleStatus : uint8_t {
    kOk = 0,
    kFail,
    kTimeout,
  };

  template <typename RspBodyType>
  using HttpHandle = std::function<Awaitable<HttpHandleStatus>(
      const Request<ReqBodyType>&,
      Response<RspBodyType>&,
      std::chrono::nanoseconds)>;

  struct Options {
    /// Listening address
    Tcp::endpoint ep = Tcp::endpoint{boost::asio::ip::address_v4(), 50080};

    /// Maximum number of connections
    size_t max_session_num = 1000000;

    /// Managing coroutine timer intervals
    std::chrono::nanoseconds mgr_timer_dt = std::chrono::seconds(5);

    /// Maximum time without data
    std::chrono::nanoseconds max_no_data_duration = std::chrono::seconds(60);

    /// Static web page path, supports relative path, requires index.html to exist in the directory.
    /// If empty, it means static web page function is not supported
    std::string doc_root;

    static Options Verify(const Options& verify_options) {
      Options options(verify_options);

      if (options.max_session_num < 1) options.max_session_num = 1;

      if (options.max_session_num > Tcp::acceptor::max_listen_connections)
        options.max_session_num = Tcp::acceptor::max_listen_connections;

      if (options.mgr_timer_dt < std::chrono::milliseconds(100))
        options.mgr_timer_dt = std::chrono::milliseconds(100);

      if (options.max_no_data_duration < std::chrono::seconds(10))
        options.max_no_data_duration = std::chrono::seconds(10);

      return options;
    }
  };

  explicit AsioHttpServer(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        acceptor_(mgr_strand_),
        acceptor_timer_(mgr_strand_),
        mgr_timer_(mgr_strand_),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()),
        http_dispatcher_ptr_(std::make_shared<Session::Dispatcher>()) {}

  ~AsioHttpServer() = default;

  AsioHttpServer(const AsioHttpServer&) = delete;
  AsioHttpServer& operator=(const AsioHttpServer&) = delete;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) {
    AIMRT_CHECK_ERROR_THROW(
        state_.load() == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    logger_ptr_ = logger_ptr;
  }

  template <typename RspBodyType = boost::beast::http::string_body>
  void RegisterHttpHandleFunc(std::string_view pattern,
                              HttpHandle<RspBodyType>&& handle) {
    AIMRT_CHECK_ERROR_THROW(
        state_.load() == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    http_dispatcher_ptr_->RegisterHttpHandle(
        pattern, Session::GenHttpDispatcherHandle(std::move(handle)));
  }

  void Initialize(const Options& options) {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    options_ = Options::Verify(options);
    session_options_ptr_ = std::make_shared<SessionOptions>(options_);

    AIMRT_CHECK_ERROR_THROW(CheckListenAddr(options_.ep),
                            "{} is already in use.", aimrt::common::util::SSToString(options_.ep));
  }

  void Start() {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, State::kStart) == State::kInit,
        "Method can only be called when state is 'Init'.");

    auto self = this->shared_from_this();
    boost::asio::co_spawn(
        mgr_strand_,
        [this, self]() -> Awaitable<void> {
          acceptor_.open(options_.ep.protocol());
          acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
          acceptor_.bind(options_.ep);
          acceptor_.listen();

          while (state_.load() == State::kStart) {
            try {
              // 如果链接数达到上限，则等待一段时间再试
              if (session_ptr_list_.size() >= options_.max_session_num) {
                acceptor_timer_.expires_after(options_.mgr_timer_dt);
                co_await acceptor_timer_.async_wait(boost::asio::use_awaitable);
                continue;
              }

              auto session_ptr = std::make_shared<Session>(
                  io_ptr_, logger_ptr_, http_dispatcher_ptr_);
              session_ptr->Initialize(session_options_ptr_);

              co_await acceptor_.async_accept(session_ptr->Socket(),
                                              boost::asio::use_awaitable);
              AIMRT_TRACE(
                  "Http svr accept a new connection, remote addr is {}",
                  aimrt::common::util::SSToString(session_ptr->Socket().remote_endpoint()));
              session_ptr->Start();

              session_ptr_list_.emplace_back(session_ptr);

            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "Http svr accept connection get exception and exit, exception info: {}",
                  e.what());
            }
          }

          Shutdown();

          co_return;
        },
        boost::asio::detached);

    boost::asio::co_spawn(
        mgr_strand_,
        [this, self]() -> Awaitable<void> {
          while (state_.load() == State::kStart) {
            try {
              mgr_timer_.expires_after(options_.mgr_timer_dt);
              co_await mgr_timer_.async_wait(boost::asio::use_awaitable);

              for (auto itr = session_ptr_list_.begin(); itr != session_ptr_list_.end();) {
                if ((*itr)->IsRunning())
                  ++itr;
                else
                  session_ptr_list_.erase(itr++);
              }
            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "Http svr timer get exception and exit, exception info: {}",
                  e.what());
            }
          }

          Shutdown();

          co_return;
        },
        boost::asio::detached);
  }

  void Shutdown() {
    if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
      return;

    auto self = this->shared_from_this();
    boost::asio::dispatch(mgr_strand_, [this, self]() {
      uint32_t stop_step = 1;
      while (stop_step) {
        try {
          switch (stop_step) {
            case 1:
              acceptor_timer_.cancel();
              ++stop_step;
            case 2:
              mgr_timer_.cancel();
              ++stop_step;
            case 3:
              acceptor_.cancel();
              ++stop_step;
            case 4:
              acceptor_.close();
              ++stop_step;
            case 5:
              // acceptor_.release(); // 使用动态库形式加载时此处会崩，待排查
              ++stop_step;
            default:
              stop_step = 0;
              break;
          }
        } catch (const std::exception& e) {
          AIMRT_WARN(
              "Http svr stop get exception at step {}, exception info: {}",
              stop_step, e.what());
          ++stop_step;
        }
      }

      for (auto& session_ptr : session_ptr_list_) session_ptr->Shutdown();

      session_ptr_list_.clear();
    });
  }

  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

 private:
  static bool CheckListenAddr(const Tcp::endpoint& ep) {
    try {
      IOCtx io;
      Tcp::acceptor acceptor(io, ep);
      return true;
    } catch (...) {
      return false;
    }
  }

  struct SessionOptions {
    explicit SessionOptions(const Options& options)
        : doc_root(options.doc_root),
          max_no_data_duration(options.max_no_data_duration) {}

    std::string doc_root;
    std::chrono::nanoseconds max_no_data_duration;
  };

  class Session : public std::enable_shared_from_this<Session> {
   public:
    using Dispatcher =
        HttpDispatcher<Awaitable<void>(const std::shared_ptr<Session>&, const Request<ReqBodyType>&)>;

    Session(const std::shared_ptr<IOCtx>& io_ptr,
            const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr,
            const std::shared_ptr<Dispatcher>& http_dispatcher_ptr)
        : io_ptr_(io_ptr),
          session_socket_strand_(boost::asio::make_strand(*io_ptr)),
          stream_(session_socket_strand_),
          session_mgr_strand_(boost::asio::make_strand(*io_ptr)),
          timer_(session_socket_strand_),
          logger_ptr_(logger_ptr),
          http_dispatcher_ptr_(http_dispatcher_ptr) {}

    ~Session() = default;

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    void Initialize(const std::shared_ptr<const SessionOptions>& session_options_ptr) {
      AIMRT_CHECK_ERROR_THROW(
          std::atomic_exchange(&state_, SessionState::kInit) == SessionState::kPreInit,
          "Method can only be called when state is 'PreInit'.");

      session_options_ptr_ = session_options_ptr;
    }

    void Start() {
      AIMRT_CHECK_ERROR_THROW(
          std::atomic_exchange(&state_, SessionState::kStart) == SessionState::kInit,
          "Method can only be called when state is 'Init'.");

      remote_addr_ = aimrt::common::util::SSToString(stream_.socket().remote_endpoint());

      auto self = this->shared_from_this();

      // Processing co
      boost::asio::co_spawn(
          session_socket_strand_,
          [this, self]() -> Awaitable<void> {
            try {
              namespace http = boost::beast::http;

              boost::beast::flat_buffer buffer;

              while (state_.load() == SessionState::kStart && !close_connect_flag_) {
                http::request_parser<ReqBodyType> req_parser;
                req_parser.eager(true);
                req_parser.body_limit(1024 * 1024 * 16);
                size_t read_data_size = co_await http::async_read(
                    stream_, buffer, req_parser, boost::asio::use_awaitable);
                AIMRT_TRACE("Http svr session async read {} bytes from {}",
                            read_data_size, RemoteAddr());
                Request<ReqBodyType> req = req_parser.release();
                tick_has_data_ = true;

                // 检查bad req
                std::string_view bad_req_check_ret = CheckBadRequest(req);
                if (!bad_req_check_ret.empty()) {
                  const auto& rsp = BadRequestHandle(req, bad_req_check_ret);
                  close_connect_flag_ = rsp.need_eof();

                  AIMRT_WARN(
                      "Http svr session get bad request, remote addr {}, err msg: {}, close_connect_flag: {}",
                      RemoteAddr(), bad_req_check_ret.data(), close_connect_flag_);
                  size_t write_data_size = co_await http::async_write(
                      stream_, rsp, boost::asio::use_awaitable);
                  AIMRT_TRACE("Http svr session async write {} bytes to {}",
                              write_data_size, RemoteAddr());
                  continue;
                }

                // 检查url
                auto url_struct = aimrt::common::util::ParseUrl(req.target());
                if (!url_struct) {
                  const auto& rsp = BadRequestHandle(req, "Can not parse url");
                  close_connect_flag_ = rsp.need_eof();

                  AIMRT_WARN(
                      "Http svr session can not parse url: {}, remote addr {}, close_connect_flag: {}",
                      RemoteAddr(), req.target().data(), close_connect_flag_);
                  size_t write_data_size = co_await http::async_write(
                      stream_, rsp, boost::asio::use_awaitable);
                  AIMRT_TRACE("Http svr session async write {} bytes to {}",
                              write_data_size, RemoteAddr());
                  continue;
                }

                // 处理handle类请求
                const auto& handle = http_dispatcher_ptr_->GetHttpHandle(url_struct->path);
                if (handle) {
                  req.set("Remote-Endpoint", RemoteAddr());  // To trace the request
                  co_await handle(self, req);
                  continue;
                }

                // 处理文件类请求
                if (session_options_ptr_->doc_root.empty()) {
                  const auto& rsp = NotFoundHandle(req, url_struct->path);
                  close_connect_flag_ = rsp.need_eof();

                  AIMRT_WARN(
                      "Http svr session get 404, path: '{}', remote addr {}, close_connect_flag: {}",
                      url_struct->path, RemoteAddr(), close_connect_flag_);
                  size_t write_data_size = co_await http::async_write(
                      stream_, rsp, boost::asio::use_awaitable);
                  AIMRT_TRACE("Http svr session async write {} bytes to {}",
                              write_data_size, RemoteAddr());
                  continue;
                }

                std::string path = PathCat(session_options_ptr_->doc_root, url_struct->path);
                if (url_struct->path.back() == '/') path.append("index.html");

                boost::beast::error_code ec;
                http::file_body::value_type body;
                body.open(path.c_str(), boost::beast::file_mode::scan, ec);

                if (ec == boost::beast::errc::no_such_file_or_directory) {
                  const auto& rsp = NotFoundHandle(req, url_struct->path);
                  close_connect_flag_ = rsp.need_eof();

                  AIMRT_WARN(
                      "Http svr session get 404, path: '{}', remote addr {}, close_connect_flag: {}",
                      url_struct->path, RemoteAddr(), close_connect_flag_);
                  size_t write_data_size = co_await http::async_write(
                      stream_, rsp, boost::asio::use_awaitable);
                  AIMRT_TRACE("Http svr session async write {} bytes to {}",
                              write_data_size, RemoteAddr());
                  continue;
                }

                if (ec) {
                  const auto& rsp = ServerErrorHandle(req, ec.message());
                  close_connect_flag_ = rsp.need_eof();

                  AIMRT_WARN(
                      "Http svr session get server error, err msg: {}, remote addr {}, close_connect_flag: {}",
                      ec.message(), RemoteAddr(), close_connect_flag_);
                  size_t write_data_size = co_await http::async_write(
                      stream_, rsp, boost::asio::use_awaitable);
                  AIMRT_TRACE("Http svr session async write {} bytes to {}",
                              write_data_size, RemoteAddr());
                  continue;
                }

                const auto size = body.size();

                // 处理head类请求
                if (req.method() == http::verb::head) {
                  Response<http::empty_body> rsp{http::status::ok, req.version()};
                  rsp.set(http::field::content_type, MimeType(path));
                  rsp.content_length(size);
                  rsp.keep_alive(req.keep_alive());

                  close_connect_flag_ = rsp.need_eof();

                  AIMRT_TRACE(
                      "Http svr session get head request, remote addr {}, close_connect_flag: {}",
                      RemoteAddr(), close_connect_flag_);
                  size_t write_data_size = co_await http::async_write(
                      stream_, rsp, boost::asio::use_awaitable);
                  AIMRT_TRACE("Http svr session async write {} bytes to {}",
                              write_data_size, RemoteAddr());
                  continue;
                }

                // 处理file请求
                Response<http::file_body> rsp{
                    std::piecewise_construct,
                    std::make_tuple(std::move(body)),
                    std::make_tuple(http::status::ok, req.version())};
                rsp.set(http::field::content_type, MimeType(path));
                rsp.content_length(size);
                rsp.keep_alive(req.keep_alive());

                close_connect_flag_ = rsp.need_eof();

                AIMRT_TRACE(
                    "Http svr session get file request, remote addr {}, close_connect_flag: {}",
                    RemoteAddr(), close_connect_flag_);
                size_t write_data_size = co_await http::async_write(
                    stream_, rsp, boost::asio::use_awaitable);
                AIMRT_TRACE("Http svr session async write {} bytes to {}",
                            write_data_size, RemoteAddr());
              }
            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "Http svr session get exception and exit, remote addr {}, exception info: {}",
                  RemoteAddr(), e.what());
            }

            Shutdown();

            co_return;
          },
          boost::asio::detached);

      // Timer co
      boost::asio::co_spawn(
          session_mgr_strand_,
          [this, self]() -> Awaitable<void> {
            try {
              while (state_.load() == SessionState::kStart) {
                timer_.expires_after(session_options_ptr_->max_no_data_duration);
                co_await timer_.async_wait(boost::asio::use_awaitable);

                if (tick_has_data_) {
                  tick_has_data_ = false;
                } else {
                  AIMRT_TRACE(
                      "Http svr session exit due to timeout({}ms), remote addr {}.",
                      std::chrono::duration_cast<std::chrono::milliseconds>(session_options_ptr_->max_no_data_duration).count(),
                      RemoteAddr());
                  break;
                }
              }
            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "Http svr session timer get exception and exit, remote addr {}, exception info: {}",
                  RemoteAddr(), e.what());
            }

            Shutdown();

            co_return;
          },
          boost::asio::detached);
    }

    void Shutdown() {
      if (std::atomic_exchange(&state_, SessionState::kShutdown) == SessionState::kShutdown)
        return;

      auto self = this->shared_from_this();
      boost::asio::dispatch(session_socket_strand_, [this, self]() {
        uint32_t stop_step = 1;
        while (stop_step) {
          try {
            switch (stop_step) {
              case 1:
                stream_.socket().shutdown(
                    boost::asio::ip::tcp::socket::shutdown_both);
                ++stop_step;
              case 2:
                stream_.socket().cancel();
                ++stop_step;
              case 3:
                stream_.socket().close();
                ++stop_step;
              case 4:
                stream_.socket().release();
                ++stop_step;
              case 5:
                stream_.cancel();
                ++stop_step;
              case 6:
                stream_.close();
                ++stop_step;
              case 7:
                stream_.release_socket();
                ++stop_step;
              default:
                stop_step = 0;
                break;
            }
          } catch (const std::exception& e) {
            AIMRT_TRACE(
                "Http svr session stop get exception at step {}, remote addr {}, exception info: {}",
                stop_step, RemoteAddr(), e.what());
            ++stop_step;
          }
        }
      });

      boost::asio::dispatch(session_mgr_strand_, [this, self]() {
        uint32_t stop_step = 1;
        while (stop_step) {
          try {
            switch (stop_step) {
              case 1:
                timer_.cancel();
                ++stop_step;
              default:
                stop_step = 0;
                break;
            }
          } catch (const std::exception& e) {
            AIMRT_WARN(
                "Http svr session mgr stop get exception at step {}, remote addr {}, exception info: {}",
                stop_step, RemoteAddr(), e.what());
            ++stop_step;
          }
        }
      });
    }

    const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

    Tcp::socket& Socket() { return stream_.socket(); }

    std::string_view RemoteAddr() const { return remote_addr_; }

    bool IsRunning() const { return state_.load() == SessionState::kStart; }

    template <typename RspBodyType = boost::beast::http::string_body>
    static Dispatcher::HttpHandle GenHttpDispatcherHandle(HttpHandle<RspBodyType>&& handle) {
      return [handle{std::move(handle)}](
                 const std::shared_ptr<Session>& session_ptr,
                 const Request<ReqBodyType>& req) -> Awaitable<void> {
        Response<RspBodyType> handle_rsp{boost::beast::http::status::ok, req.version()};
        HttpHandleStatus handle_status;

        auto handle_timeout = std::chrono::seconds::max();
        auto req_timeout_itr = req.find(boost::beast::http::field::timeout);
        if (req_timeout_itr != req.end()) {
          auto timeout_str = req_timeout_itr->value();
          if (aimrt::common::util::IsDigitStr(timeout_str))
            handle_timeout = std::chrono::seconds(atoi(timeout_str.data()));
        }

        std::string err_info;
        try {
          handle_status = co_await boost::asio::co_spawn(
              *(session_ptr->io_ptr_),
              [&]() -> Awaitable<AsioHttpServer::HttpHandleStatus> {
                return handle(req, handle_rsp, handle_timeout);
              },
              boost::asio::use_awaitable);

          if (handle_status != HttpHandleStatus::kOk)
            err_info = "handle failed, status: " + std::to_string(static_cast<uint8_t>(handle_status));
        } catch (const std::exception& e) {
          err_info = e.what();
        }

        if (!err_info.empty()) [[unlikely]] {
          const auto& rsp = ServerErrorHandle(req, err_info);
          session_ptr->close_connect_flag_ = rsp.need_eof();

          AIMRT_HL_WARN(
              session_ptr->GetLogger(),
              "Http svr session custom handle request get exp, error info: {}, remote addr {}, close_connect_flag: {}",
              err_info, session_ptr->RemoteAddr(), session_ptr->close_connect_flag_);

          co_await boost::beast::http::async_write(
              session_ptr->stream_, rsp, boost::asio::use_awaitable);
          AIMRT_HL_TRACE(
              session_ptr->GetLogger(),
              "Http svr session async write to {} suc.",
              session_ptr->RemoteAddr());

          co_return;
        }

        session_ptr->close_connect_flag_ = handle_rsp.need_eof();

        AIMRT_HL_TRACE(
            session_ptr->GetLogger(),
            "Http svr session custom handle request, remote addr {}, close_connect_flag: {}",
            session_ptr->RemoteAddr(), session_ptr->close_connect_flag_);

        co_await boost::beast::http::async_write(
            session_ptr->stream_, handle_rsp, boost::asio::use_awaitable);

        AIMRT_HL_TRACE(
            session_ptr->GetLogger(),
            "Http svr session async write to {} suc.",
            session_ptr->RemoteAddr());

        co_return;
      };
    }

   private:
    static std::string_view MimeType(std::string_view path) {
      using boost::beast::iequals;
      auto const ext = [&path] {
        auto const pos = path.rfind('.');
        if (pos == std::string_view::npos)
          return std::string_view();
        return path.substr(pos);
      }();

      if (iequals(ext, ".htm")) return "text/html";
      if (iequals(ext, ".html")) return "text/html";
      if (iequals(ext, ".php")) return "text/html";
      if (iequals(ext, ".css")) return "text/css";
      if (iequals(ext, ".txt")) return "text/plain";
      if (iequals(ext, ".js")) return "application/javascript";
      if (iequals(ext, ".json")) return "application/json";
      if (iequals(ext, ".xml")) return "application/xml";
      if (iequals(ext, ".swf")) return "application/x-shockwave-flash";
      if (iequals(ext, ".flv")) return "video/x-flv";
      if (iequals(ext, ".png")) return "image/png";
      if (iequals(ext, ".jpe")) return "image/jpeg";
      if (iequals(ext, ".jpeg")) return "image/jpeg";
      if (iequals(ext, ".jpg")) return "image/jpeg";
      if (iequals(ext, ".gif")) return "image/gif";
      if (iequals(ext, ".bmp")) return "image/bmp";
      if (iequals(ext, ".ico")) return "image/vnd.microsoft.icon";
      if (iequals(ext, ".tiff")) return "image/tiff";
      if (iequals(ext, ".tif")) return "image/tiff";
      if (iequals(ext, ".svg")) return "image/svg+xml";
      if (iequals(ext, ".svgz")) return "image/svg+xml";
      return "application/text";
    }

    static std::string PathCat(std::string_view base, std::string_view path) {
      if (base.empty())
        return std::string(path);
      std::string result(base);
#ifdef _WIN32
      char constexpr path_separator = '\\';
      if (result.back() == path_separator)
        result.resize(result.size() - 1);
      result.append(path.data(), path.size());
      for (auto& c : result)
        if (c == '/')
          c = path_separator;
#else
      char constexpr kPathSeparator = '/';
      if (result.back() == kPathSeparator)
        result.resize(result.size() - 1);
      result.append(path.data(), path.size());
#endif
      return result;
    }

    static std::string_view CheckBadRequest(const Request<ReqBodyType>& req) {
      namespace http = boost::beast::http;

      // HTTP method 检查
      if (req.method() != http::verb::get && req.method() != http::verb::head &&
          req.method() != http::verb::post) {
        return "UnSupport HTTP-method";
      }

      // uri 检查
      if (req.target().empty() ||
          req.target()[0] != '/' ||
          req.target().find("..") != std::string_view::npos) {
        return "Illegal request-target";
      }

      return "";
    }

    static Response<boost::beast::http::string_body> BadRequestHandle(
        const Request<ReqBodyType>& req, std::string_view info) {
      namespace http = boost::beast::http;

      Response<http::string_body> rsp{http::status::bad_request, req.version()};
      rsp.set(http::field::content_type, "text/html");
      rsp.keep_alive(req.keep_alive());
      rsp.body() = info;
      rsp.prepare_payload();
      return rsp;
    }

    static Response<boost::beast::http::string_body> NotFoundHandle(
        const Request<ReqBodyType>& req, std::string_view info) {
      namespace http = boost::beast::http;

      Response<http::string_body> rsp{http::status::not_found, req.version()};
      rsp.set(http::field::content_type, "text/html");
      rsp.keep_alive(req.keep_alive());
      rsp.body() = "The resource '" + std::string(info) + "' was not found.";
      rsp.prepare_payload();
      return rsp;
    }

    static Response<boost::beast::http::string_body> ServerErrorHandle(
        const Request<ReqBodyType>& req, std::string_view info) {
      namespace http = boost::beast::http;

      Response<http::string_body> rsp{http::status::internal_server_error, req.version()};
      rsp.set(http::field::content_type, "text/html");
      rsp.keep_alive(req.keep_alive());
      rsp.body() = "An error occurred: '" + std::string(info) + "'";
      rsp.prepare_payload();
      return rsp;
    }

   private:
    enum class SessionState : uint32_t {
      kPreInit,
      kInit,
      kStart,
      kShutdown,
    };

    // IO CTX
    std::shared_ptr<IOCtx> io_ptr_;
    Strand session_socket_strand_;
    boost::beast::tcp_stream stream_;
    Strand session_mgr_strand_;
    Timer timer_;

    // Log handle
    std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

    // Dispatcher
    std::shared_ptr<Dispatcher> http_dispatcher_ptr_;

    // Options
    std::shared_ptr<const SessionOptions> session_options_ptr_;

    // State
    std::atomic<SessionState> state_ = SessionState::kPreInit;

    // misc
    std::string remote_addr_;
    std::atomic_bool tick_has_data_ = false;
    bool close_connect_flag_ = false;
  };

 private:
  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

  // IO CTX
  std::shared_ptr<IOCtx> io_ptr_;
  Strand mgr_strand_;       // Session pool operation strand
  Tcp::acceptor acceptor_;  // Listener
  Timer acceptor_timer_;    // The sleep timer when the connection is full
  Timer mgr_timer_;         // Timer to manage session pool

  // Log handle
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  // Dispatcher
  std::shared_ptr<Session::Dispatcher> http_dispatcher_ptr_;

  // Options
  Options options_;

  // State
  std::atomic<State> state_ = State::kPreInit;

  // Session management
  std::shared_ptr<const SessionOptions> session_options_ptr_;
  std::list<std::shared_ptr<Session>> session_ptr_list_;  // session池
};

}  // namespace aimrt::common::net