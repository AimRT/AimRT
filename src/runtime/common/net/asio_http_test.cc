// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "net/asio_http_cli.h"
#include "net/asio_http_svr.h"
#include "net/asio_tools.h"
#include "util/string_util.h"

namespace aimrt::runtime::common::net {

namespace asio = boost::asio;
namespace http = boost::beast::http;

inline aimrt::common::util::SimpleLogger& GetLogger() {
  static aimrt::common::util::SimpleLogger logger;
  return logger;
}

TEST(NET_TEST, Http_base) {
  auto cli_sys_ptr = std::make_shared<AsioExecutor>(2);
  auto svr_sys_ptr = std::make_shared<AsioExecutor>(2);

  // start svr
  std::thread t_svr([svr_sys_ptr] {
    AIMRT_INFO("svr_sys_ptr start.");
    auto http_svr_ptr = std::make_shared<AsioHttpServer>(svr_sys_ptr->IO());
    http_svr_ptr->Initialize(AsioHttpServer::Options{
        .ep = {asio::ip::address_v4(), 50080}});

    svr_sys_ptr->RegisterSvrFunc([http_svr_ptr] { http_svr_ptr->Start(); },
                                 [http_svr_ptr] { http_svr_ptr->Shutdown(); });

    svr_sys_ptr->Start();
    svr_sys_ptr->Join();
    AIMRT_INFO("svr_sys_ptr exit.");
  });

  // start cli
  auto http_cli_pool_ptr = std::make_shared<AsioHttpClientPool>(cli_sys_ptr->IO());
  http_cli_pool_ptr->Initialize(AsioHttpClientPool::Options{});

  cli_sys_ptr->RegisterSvrFunc([http_cli_pool_ptr] { http_cli_pool_ptr->Start(); },
                               [http_cli_pool_ptr] { http_cli_pool_ptr->Shutdown(); });

  std::thread t_cli([cli_sys_ptr] {
    AIMRT_INFO("cli_sys_ptr start.");
    cli_sys_ptr->Start();
    cli_sys_ptr->Join();
    AIMRT_INFO("cli_sys_ptr exit.");
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // call http
  auto http_send_recv =
      [http_cli_pool_ptr](AsioHttpClient::Options client_options, bool expect_exp = false)
      -> asio::awaitable<void> {
    bool exp_flag = false;
    try {
      auto client_ptr = co_await http_cli_pool_ptr->GetClient(client_options);

      http::request<http::string_body> req{http::verb::get, "/", 11};
      req.set(http::field::host, "127.0.0.1");
      req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

      AIMRT_INFO("req:\n{}", aimrt::common::util::SSToString(req));

      auto rsp = co_await client_ptr->HttpSendRecvCo(req);

      AIMRT_INFO("rsp:\n{}", aimrt::common::util::SSToString(rsp));

      // check rsp
      EXPECT_EQ(rsp.result_int(), 404);
      EXPECT_EQ(rsp.result(), http::status::not_found);

      auto rsp_reason = rsp.reason();
      EXPECT_STREQ(std::string(rsp_reason.data(), rsp_reason.size()).c_str(), "Not Found");

      auto rsp_content_length = rsp.at(http::field::content_length);
      EXPECT_STREQ(std::string(rsp_content_length.data(), rsp_content_length.size()).c_str(), "31");

      auto rsp_content_type = rsp.at(http::field::content_type);
      EXPECT_STREQ(std::string(rsp_content_type.data(), rsp_content_type.size()).c_str(), "text/html");

      EXPECT_EQ(rsp.body().size(), 31);
      EXPECT_STREQ(rsp.body().c_str(), "The resource '/' was not found.");

    } catch (const std::exception& e) {
      AIMRT_INFO("http_send_recv_co get exception and exit, exception: {}", e.what());
      exp_flag = true;
    }
    EXPECT_EQ(exp_flag, expect_exp);
    co_return;
  };

  auto co_future_1 = asio::co_spawn(
      *(cli_sys_ptr->IO()),
      http_send_recv(AsioHttpClient::Options{"127.0.0.1", "50080"}),
      asio::use_future);
  auto co_future_2 = asio::co_spawn(
      *(cli_sys_ptr->IO()),
      http_send_recv(AsioHttpClient::Options{"127.0.0.1", "50080"}),
      asio::use_future);

  co_future_1.wait();
  co_future_2.wait();

  // stop cli
  cli_sys_ptr->Shutdown();
  t_cli.join();

  // stop svr
  svr_sys_ptr->Shutdown();
  t_svr.join();
}

TEST(NET_TEST, Http_multi_server) {
  auto cli_sys_ptr = std::make_shared<AsioExecutor>(2);
  auto svr1_sys_ptr = std::make_shared<AsioExecutor>(2);
  auto svr2_sys_ptr = std::make_shared<AsioExecutor>(2);

  // start svr1
  std::thread t_svr1([svr1_sys_ptr] {
    AIMRT_INFO("svr1_sys_ptr start.");
    auto http_svr_ptr = std::make_shared<AsioHttpServer>(svr1_sys_ptr->IO());
    http_svr_ptr->Initialize(AsioHttpServer::Options{
        .ep = {asio::ip::address_v4(), 50080}});

    svr1_sys_ptr->RegisterSvrFunc([http_svr_ptr] { http_svr_ptr->Start(); },
                                  [http_svr_ptr] { http_svr_ptr->Shutdown(); });

    svr1_sys_ptr->Start();
    svr1_sys_ptr->Join();
    AIMRT_INFO("svr1_sys_ptr exit.");
  });

  // start svr2
  std::thread t_svr2([svr2_sys_ptr] {
    AIMRT_INFO("svr2_sys_ptr start.");
    auto http_svr_ptr = std::make_shared<AsioHttpServer>(svr2_sys_ptr->IO());
    http_svr_ptr->Initialize(AsioHttpServer::Options{
        .ep = {asio::ip::address_v4(), 50081}});

    svr2_sys_ptr->RegisterSvrFunc([http_svr_ptr] { http_svr_ptr->Start(); },
                                  [http_svr_ptr] { http_svr_ptr->Shutdown(); });

    svr2_sys_ptr->Start();
    svr2_sys_ptr->Join();
    AIMRT_INFO("svr2_sys_ptr exit.");
  });

  // start cli
  auto http_cli_pool_ptr = std::make_shared<AsioHttpClientPool>(cli_sys_ptr->IO());
  http_cli_pool_ptr->Initialize(AsioHttpClientPool::Options{});

  cli_sys_ptr->RegisterSvrFunc([http_cli_pool_ptr] { http_cli_pool_ptr->Start(); },
                               [http_cli_pool_ptr] { http_cli_pool_ptr->Shutdown(); });

  std::thread t_cli([cli_sys_ptr] {
    AIMRT_INFO("cli_sys_ptr start.");
    cli_sys_ptr->Start();
    cli_sys_ptr->Join();
    AIMRT_INFO("cli_sys_ptr exit.");
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // call http
  auto http_send_recv =
      [http_cli_pool_ptr](AsioHttpClient::Options client_options, bool expect_exp = false)
      -> asio::awaitable<void> {
    bool exp_flag = false;
    try {
      auto client_ptr = co_await http_cli_pool_ptr->GetClient(client_options);
      auto client_ptr2 = co_await http_cli_pool_ptr->GetClient(client_options);
      EXPECT_EQ(client_ptr, client_ptr2);

      http::request<http::string_body> req{http::verb::get, "/", 11};
      req.set(http::field::host, "127.0.0.1");
      req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

      AIMRT_INFO("req:\n{}", aimrt::common::util::SSToString(req));

      auto rsp = co_await client_ptr->HttpSendRecvCo(req);

      AIMRT_INFO("rsp:\n{}", aimrt::common::util::SSToString(rsp));

      // check rsp
      EXPECT_EQ(rsp.result_int(), 404);
      EXPECT_EQ(rsp.result(), http::status::not_found);

      auto rsp_reason = rsp.reason();
      EXPECT_STREQ(std::string(rsp_reason.data(), rsp_reason.size()).c_str(), "Not Found");

      auto rsp_content_length = rsp.at(http::field::content_length);
      EXPECT_STREQ(std::string(rsp_content_length.data(), rsp_content_length.size()).c_str(), "31");

      auto rsp_content_type = rsp.at(http::field::content_type);
      EXPECT_STREQ(std::string(rsp_content_type.data(), rsp_content_type.size()).c_str(), "text/html");

      EXPECT_EQ(rsp.body().size(), 31);
      EXPECT_STREQ(rsp.body().c_str(), "The resource '/' was not found.");

    } catch (const std::exception& e) {
      AIMRT_INFO("http_send_recv_co get exception and exit, exception: {}", e.what());
      exp_flag = true;
    }
    EXPECT_EQ(exp_flag, expect_exp);
    co_return;
  };

  auto co_future_1 = asio::co_spawn(
      *(cli_sys_ptr->IO()),
      http_send_recv(AsioHttpClient::Options{"127.0.0.1", "50080"}),
      asio::use_future);
  auto co_future_2 = asio::co_spawn(
      *(cli_sys_ptr->IO()),
      http_send_recv(AsioHttpClient::Options{"127.0.0.1", "50081"}),
      asio::use_future);

  co_future_1.wait();
  co_future_2.wait();

  // stop cli
  cli_sys_ptr->Shutdown();
  t_cli.join();

  // stop svr1
  svr1_sys_ptr->Shutdown();
  t_svr1.join();

  // stop svr2
  svr2_sys_ptr->Shutdown();
  t_svr2.join();
}

TEST(NET_TEST, Http_server_restart) {
  auto cli_sys_ptr = std::make_shared<AsioExecutor>(2);
  auto svr1_sys_ptr = std::make_shared<AsioExecutor>(2);
  auto svr2_sys_ptr = std::make_shared<AsioExecutor>(2);

  // start cli
  auto http_cli_ptr = std::make_shared<AsioHttpClient>(cli_sys_ptr->IO());
  http_cli_ptr->Initialize(AsioHttpClient::Options{"127.0.0.1", "50080"});

  cli_sys_ptr->RegisterSvrFunc([http_cli_ptr] { http_cli_ptr->Start(); },
                               [http_cli_ptr] { http_cli_ptr->Shutdown(); });

  std::thread t_cli([cli_sys_ptr] {
    AIMRT_INFO("cli_sys_ptr start.");
    cli_sys_ptr->Start();
    cli_sys_ptr->Join();
    AIMRT_INFO("cli_sys_ptr exit.");
  });

  auto http_send_recv =
      [http_cli_ptr](bool expect_exp = false) -> asio::awaitable<void> {
    bool exp_flag = false;
    try {
      http::request<http::string_body> req{http::verb::get, "/", 11};
      req.set(http::field::host, "127.0.0.1");
      req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

      AIMRT_INFO("req:\n{}", aimrt::common::util::SSToString(req));

      auto rsp = co_await http_cli_ptr->HttpSendRecvCo(req);

      AIMRT_INFO("rsp:\n{}", aimrt::common::util::SSToString(rsp));

      // check rsp
      EXPECT_EQ(rsp.result_int(), 404);
      EXPECT_EQ(rsp.result(), http::status::not_found);

      auto rsp_reason = rsp.reason();
      EXPECT_STREQ(std::string(rsp_reason.data(), rsp_reason.size()).c_str(), "Not Found");

      auto rsp_content_length = rsp.at(http::field::content_length);
      EXPECT_STREQ(std::string(rsp_content_length.data(), rsp_content_length.size()).c_str(), "31");

      auto rsp_content_type = rsp.at(http::field::content_type);
      EXPECT_STREQ(std::string(rsp_content_type.data(), rsp_content_type.size()).c_str(), "text/html");

      EXPECT_EQ(rsp.body().size(), 31);
      EXPECT_STREQ(rsp.body().c_str(), "The resource '/' was not found.");

    } catch (const std::exception& e) {
      AIMRT_INFO("http_send_recv_co get exception and exit, exception: {}", e.what());
      exp_flag = true;
    }
    EXPECT_EQ(exp_flag, expect_exp);
    co_return;
  };

  // start svr1
  std::thread t_svr1([svr1_sys_ptr] {
    AIMRT_INFO("svr1_sys_ptr start.");
    auto http_svr_ptr = std::make_shared<AsioHttpServer>(svr1_sys_ptr->IO());
    http_svr_ptr->Initialize(AsioHttpServer::Options{
        .ep = {asio::ip::address_v4(), 50080}});

    svr1_sys_ptr->RegisterSvrFunc([http_svr_ptr] { http_svr_ptr->Start(); },
                                  [http_svr_ptr] { http_svr_ptr->Shutdown(); });

    svr1_sys_ptr->Start();
    svr1_sys_ptr->Join();
    AIMRT_INFO("svr1_sys_ptr exit.");
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // call http
  auto co_future_1_1 = asio::co_spawn(
      *(cli_sys_ptr->IO()), http_send_recv(), asio::use_future);
  auto co_future_1_2 = asio::co_spawn(
      *(cli_sys_ptr->IO()), http_send_recv(), asio::use_future);
  co_future_1_1.wait();
  co_future_1_2.wait();

  auto co_future_1_3 = asio::co_spawn(
      *(cli_sys_ptr->IO()), http_send_recv(), asio::use_future);
  auto co_future_1_4 = asio::co_spawn(
      *(cli_sys_ptr->IO()), http_send_recv(), asio::use_future);
  co_future_1_3.wait();
  co_future_1_4.wait();

  // stop svr1
  svr1_sys_ptr->Shutdown();
  t_svr1.join();

  // start svr2
  std::thread t_svr2([svr2_sys_ptr] {
    AIMRT_INFO("svr2_sys_ptr start.");
    auto http_svr_ptr = std::make_shared<AsioHttpServer>(svr2_sys_ptr->IO());
    http_svr_ptr->Initialize(AsioHttpServer::Options{
        .ep = {asio::ip::address_v4(), 50080}});

    svr2_sys_ptr->RegisterSvrFunc([http_svr_ptr] { http_svr_ptr->Start(); },
                                  [http_svr_ptr] { http_svr_ptr->Shutdown(); });
    svr2_sys_ptr->Start();
    svr2_sys_ptr->Join();
    AIMRT_INFO("svr2_sys_ptr exit.");
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // call http
  // 服务端重启，客户端这边之前的session都会失效，请求会抛异常
  auto co_future_2_1 = asio::co_spawn(
      *(cli_sys_ptr->IO()), http_send_recv(true), asio::use_future);
  auto co_future_2_2 = asio::co_spawn(
      *(cli_sys_ptr->IO()), http_send_recv(true), asio::use_future);
  co_future_2_1.wait();
  co_future_2_2.wait();

  auto co_future_2_3 = asio::co_spawn(
      *(cli_sys_ptr->IO()), http_send_recv(), asio::use_future);
  co_future_2_3.wait();

  // stop svr2
  svr2_sys_ptr->Shutdown();
  t_svr2.join();

  // stop cli
  cli_sys_ptr->Shutdown();
  t_cli.join();
}

TEST(NET_TEST, Http_server_handle) {
  auto cli_sys_ptr = std::make_shared<AsioExecutor>(2);
  auto svr_sys_ptr = std::make_shared<AsioExecutor>(2);

  // start cli
  auto http_cli_ptr = std::make_shared<AsioHttpClient>(cli_sys_ptr->IO());
  http_cli_ptr->Initialize(AsioHttpClient::Options{"127.0.0.1", "50080"});

  cli_sys_ptr->RegisterSvrFunc([http_cli_ptr] { http_cli_ptr->Start(); },
                               [http_cli_ptr] { http_cli_ptr->Shutdown(); });

  std::thread t_cli([cli_sys_ptr] {
    AIMRT_INFO("cli_sys_ptr start.");
    cli_sys_ptr->Start();
    cli_sys_ptr->Join();
    AIMRT_INFO("cli_sys_ptr exit.");
  });

  // start svr
  AsioHttpServer::HttpHandle<http::string_body> http_handle =
      [](const http::request<http::dynamic_body>& req,
         http::response<http::string_body>& rsp,
         std::chrono::nanoseconds timeout)
      -> asio::awaitable<AsioHttpServer::HttpHandleStatus> {
    AIMRT_INFO("handle req:\n{}", aimrt::common::util::SSToString(req));

    rsp = http::response<http::string_body>{http::status::ok, req.version()};
    rsp.set(http::field::server, BOOST_BEAST_VERSION_STRING);
    rsp.set(http::field::content_type, "text/html");
    rsp.keep_alive(req.keep_alive());
    rsp.body() = "echo: " + boost::beast::buffers_to_string(req.body().data());
    rsp.prepare_payload();

    AIMRT_INFO("handle rsp:\n{}", aimrt::common::util::SSToString(rsp));

    co_return AsioHttpServer::HttpHandleStatus::kOk;
  };

  auto http_svr_ptr = std::make_shared<AsioHttpServer>(svr_sys_ptr->IO());
  http_svr_ptr->RegisterHttpHandleFunc<http::string_body>("/test/.*", std::move(http_handle));
  http_svr_ptr->Initialize(AsioHttpServer::Options{
      .ep = {asio::ip::address_v4(), 50080}});

  svr_sys_ptr->RegisterSvrFunc([http_svr_ptr] { http_svr_ptr->Start(); },
                               [http_svr_ptr] { http_svr_ptr->Shutdown(); });

  std::thread t_svr([svr_sys_ptr] {
    AIMRT_INFO("svr_sys_ptr start.");
    svr_sys_ptr->Start();
    svr_sys_ptr->Join();
    AIMRT_INFO("svr_sys_ptr exit.");
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // call http
  auto http_send_recv =
      [http_cli_ptr](std::string msg, bool expect_exp = false) -> asio::awaitable<void> {
    bool exp_flag = false;
    try {
      http::request<http::string_body> req{http::verb::post, "/test/xxx?key1=val1&key2=val2", 11};
      req.set(http::field::host, "127.0.0.1");
      req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
      req.body() = msg;
      req.keep_alive(false);
      req.prepare_payload();

      AIMRT_INFO("req:\n{}", aimrt::common::util::SSToString(req));

      auto rsp = co_await http_cli_ptr->HttpSendRecvCo(req);

      AIMRT_INFO("rsp:\n{}", aimrt::common::util::SSToString(rsp));

      // check rsp
      EXPECT_EQ(rsp.result_int(), 200);
      EXPECT_EQ(rsp.result(), http::status::ok);

      auto rsp_reason = rsp.reason();
      EXPECT_STREQ(std::string(rsp_reason.data(), rsp_reason.size()).c_str(), "OK");

      auto rsp_content_type = rsp.at(http::field::content_type);
      EXPECT_STREQ(std::string(rsp_content_type.data(), rsp_content_type.size()).c_str(), "text/html");

      EXPECT_STREQ(rsp.body().c_str(), ("echo: " + msg).c_str());

    } catch (const std::exception& e) {
      AIMRT_INFO("http_send_recv_co get exception and exit, exception: {}", e.what());
      exp_flag = true;
    }
    EXPECT_EQ(exp_flag, expect_exp);
    co_return;
  };
  auto co_future_1 = asio::co_spawn(
      *(cli_sys_ptr->IO()), http_send_recv("msg11111111"), asio::use_future);
  co_future_1.wait();

  auto co_future_2 = asio::co_spawn(
      *(cli_sys_ptr->IO()), http_send_recv("msg2222222"), asio::use_future);
  co_future_2.wait();

  // stop svr
  svr_sys_ptr->Shutdown();
  t_svr.join();

  // stop cli
  cli_sys_ptr->Shutdown();
  t_cli.join();
}

}  // namespace aimrt::runtime::common::net
