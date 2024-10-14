// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "net/asio_tools.h"
#include "net/asio_udp_cli.h"
#include "net/asio_udp_svr.h"
#include "util/string_util.h"

namespace aimrt::common::net {

namespace asio = boost::asio;

inline aimrt::common::util::SimpleLogger &GetLogger() {
  static aimrt::common::util::SimpleLogger logger;
  return logger;
}

TEST(NET_TEST, UDP_base) {
  auto cli_sys_ptr = std::make_shared<AsioExecutor>(2);
  auto svr_sys_ptr = std::make_shared<AsioExecutor>(2);

  std::string result_str;

  // start cli
  auto udp_cli_ptr = std::make_shared<AsioUdpClient>(cli_sys_ptr->IO());
  udp_cli_ptr->Initialize(AsioUdpClient::Options{
      .svr_ep = asio::ip::udp::endpoint{asio::ip::address_v4({127, 0, 0, 1}), 53927}});

  cli_sys_ptr->RegisterSvrFunc([udp_cli_ptr] { udp_cli_ptr->Start(); },
                               [udp_cli_ptr] { udp_cli_ptr->Shutdown(); });

  std::thread t_cli([cli_sys_ptr] {
    AIMRT_INFO("cli_sys_ptr start.");
    cli_sys_ptr->Start();
    cli_sys_ptr->Join();
    AIMRT_INFO("cli_sys_ptr exit.");
  });

  // start svr
  std::thread t_svr([svr_sys_ptr, &result_str] {
    AIMRT_INFO("svr_sys_ptr start.");
    auto udp_svr_ptr = std::make_shared<AsioUdpServer>(svr_sys_ptr->IO());
    udp_svr_ptr->RegisterMsgHandle(
        [&result_str](const asio::ip::udp::endpoint &ep,
                      const std::shared_ptr<asio::streambuf> &msg_buf_ptr) {
          result_str = std::string(
              static_cast<const char *>(msg_buf_ptr->data().data()),
              msg_buf_ptr->size());
          AIMRT_INFO("svr get a msg from {}, size: {}, data: {}",
                     aimrt::common::util::SSToString(ep), msg_buf_ptr->size(), result_str);
        });
    udp_svr_ptr->Initialize(AsioUdpServer::Options{});

    svr_sys_ptr->RegisterSvrFunc([udp_svr_ptr] { udp_svr_ptr->Start(); },
                                 [udp_svr_ptr] { udp_svr_ptr->Shutdown(); });

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
  udp_cli_ptr->SendMsg(msg_buf_ptr);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_STREQ(result_str.c_str(), "12345678abcdefgh");

  // shutdown
  svr_sys_ptr->Shutdown();
  t_svr.join();

  cli_sys_ptr->Shutdown();
  t_cli.join();
}

}  // namespace aimrt::common::net
