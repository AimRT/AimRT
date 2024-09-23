// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "net/asio_tcp_cli.h"
#include "net/asio_tcp_svr.h"
#include "net/asio_tools.h"

namespace aimrt::runtime::common::net {

namespace asio = boost::asio;

inline aimrt::common::util::SimpleLogger &GetLogger() {
  static aimrt::common::util::SimpleLogger logger;
  return logger;
}

TEST(NET_TEST, Tcp_base) {
  auto cli_sys_ptr = std::make_shared<AsioExecutor>(2);
  auto svr_sys_ptr = std::make_shared<AsioExecutor>(2);

  std::string cli_result_str;
  std::string svr_result_str;

  // start cli
  auto tcp_cli_ptr = std::make_shared<AsioTcpClient>(cli_sys_ptr->IO());
  tcp_cli_ptr->RegisterMsgHandle(
      [&cli_result_str](const std::shared_ptr<asio::streambuf> &msg_buf_ptr) {
        cli_result_str = std::string(
            static_cast<const char *>(msg_buf_ptr->data().data()), msg_buf_ptr->size());
        AIMRT_INFO("cli get a msg, size: {}, data: {}", cli_result_str.size(), cli_result_str);
      });

  tcp_cli_ptr->Initialize(AsioTcpClient::Options{
      .svr_ep = asio::ip::tcp::endpoint{asio::ip::address_v4({127, 0, 0, 1}), 57634}});

  cli_sys_ptr->RegisterSvrFunc([tcp_cli_ptr] { tcp_cli_ptr->Start(); },
                               [tcp_cli_ptr] { tcp_cli_ptr->Shutdown(); });

  std::thread t_cli([cli_sys_ptr] {
    AIMRT_INFO("cli_sys_ptr start.");
    cli_sys_ptr->Start();
    cli_sys_ptr->Join();
    AIMRT_INFO("cli_sys_ptr exit.");
  });

  // start svr
  std::thread t_svr([svr_sys_ptr, &svr_result_str] {
    AIMRT_INFO("svr_sys_ptr start.");
    auto tcp_svr_ptr = std::make_shared<AsioTcpServer>(svr_sys_ptr->IO());
    tcp_svr_ptr->RegisterMsgHandle(
        [tcp_svr_ptr, &svr_result_str](const asio::ip::tcp::endpoint &ep,
                                       const std::shared_ptr<asio::streambuf> &msg_buf_ptr) {
          svr_result_str = std::string(
              static_cast<const char *>(msg_buf_ptr->data().data()), msg_buf_ptr->size());
          AIMRT_INFO("svr get a msg from {}, size: {}, data: {}",
                     aimrt::common::util::SSToString(ep), svr_result_str.size(), svr_result_str);
          tcp_svr_ptr->SendMsg(ep, msg_buf_ptr);
        });

    tcp_svr_ptr->Initialize(AsioTcpServer::Options{
        .ep = {asio::ip::address_v4(), 57634}});

    svr_sys_ptr->RegisterSvrFunc([tcp_svr_ptr] { tcp_svr_ptr->Start(); },
                                 [tcp_svr_ptr] { tcp_svr_ptr->Shutdown(); });

    svr_sys_ptr->Start();
    svr_sys_ptr->Join();
    AIMRT_INFO("svr_sys_ptr exit.");
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // send msg
  auto msg_buf_ptr = std::make_shared<asio::streambuf>();
  auto buf = msg_buf_ptr->prepare(20);
  sprintf(static_cast<char *>(buf.data()), "1234567890123456789");
  msg_buf_ptr->commit(8);

  auto buf2 = msg_buf_ptr->prepare(20);
  sprintf(static_cast<char *>(buf2.data()), "abcdefghijklmn");
  msg_buf_ptr->commit(8);
  tcp_cli_ptr->SendMsg(msg_buf_ptr);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_STREQ(cli_result_str.c_str(), "12345678abcdefgh");
  EXPECT_STREQ(svr_result_str.c_str(), "12345678abcdefgh");

  // shutdown
  svr_sys_ptr->Shutdown();
  t_svr.join();

  cli_sys_ptr->Shutdown();
  t_cli.join();
}

}  // namespace aimrt::runtime::common::net
