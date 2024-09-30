// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <csignal>
#include <fstream>
#include <iostream>

#include "gflags/gflags.h"

#include "core/aimrt_core.h"
#include "core/util/version.h"

DEFINE_string(cfg_file_path, "", "config file path");

DEFINE_bool(dump_cfg_file, false, "dump config file");
DEFINE_string(dump_cfg_file_path, "./dump_cfg.yaml", "dump config file path");

DEFINE_bool(dump_init_report, false, "dump init report");
DEFINE_string(dump_init_report_path, "./init_report.txt", "dump init report path");

DEFINE_bool(register_signal, true, "register handle for sigint and sigterm");
DEFINE_int32(running_duration, 0, "running duration, seconds");

DEFINE_bool(h, false, "help");
DEFINE_bool(v, false, "version");
DECLARE_bool(help);
DECLARE_bool(helpshort);
DECLARE_bool(version);

using namespace aimrt::runtime::core;

AimRTCore* global_core_ptr = nullptr;

void SignalHandler(int sig) {
  if (global_core_ptr && (sig == SIGINT || sig == SIGTERM)) {
    global_core_ptr->Shutdown();
    return;
  }

  raise(sig);
};

void PrintVersion() {
  std::cout << "AimRT Version: " << util::GetAimRTVersion() << std::endl;
}

void PrintUsage() {
  std::cout << "OVERVIEW: AimRT is a high-performance runtime framework for modern robotics.\n\n"
               "VERSION: "
            << util::GetAimRTVersion()
            << "\n\nUSAGE: aimrt_main --cfg_file_path=<string> [options]\n\n"
               "OPTIONS:\n\n"
               "Generic Options:\n\n"
               "  --version                         - Show version\n"
               "  --help                            - Show help\n\n"
               "AimRT Options:\n\n"
               "  --cfg_file_path=<string>          - Path to the configuration file, default is empty\n"
               "  --dump_cfg_file                   - Dump the configuration file to a file, default is false\n"
               "  --dump_cfg_file_path=<string>     - Path to dump the configuration file, default is ./dump_cfg.yaml\n"
               "  --dump_init_report                - Dump the initialization report, default is false\n"
               "  --dump_init_report_path=<string>  - Path to dump the initialization report, default is ./init_report.txt\n"
               "  --register_signal                 - Register signal handlers for SIGINT and SIGTERM, default is true\n"
               "  --running_duration=<int>          - Running duration in seconds, default is 0 which means running forever\n";
}

void HandleCommandLineFlags(int32_t argc, char** argv) {
  if (argc == 1) {
    PrintUsage();
    exit(0);
  }
  gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
  if (FLAGS_version || FLAGS_v) {
    PrintVersion();
    exit(0);
  }
  if (FLAGS_help || FLAGS_h) {
    PrintUsage();
    exit(0);
  }
}

int32_t main(int32_t argc, char** argv) {
  HandleCommandLineFlags(argc, argv);

  if (FLAGS_register_signal) {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
  }

  std::cout << "AimRT start." << std::endl;
  PrintVersion();

  try {
    AimRTCore core;
    global_core_ptr = &core;

    AimRTCore::Options options;
    options.cfg_file_path = FLAGS_cfg_file_path;

    core.Initialize(options);

    if (FLAGS_dump_cfg_file) {
      std::ofstream ofs(FLAGS_dump_cfg_file_path, std::ios::trunc);
      ofs << core.GetConfiguratorManager().GetRootOptionsNode();
      ofs.close();
    }

    if (FLAGS_dump_init_report) {
      std::ofstream ofs(FLAGS_dump_init_report_path, std::ios::trunc);
      ofs << core.GenInitializationReport();
      ofs.close();
    }

    if (FLAGS_running_duration == 0) {
      core.Start();

      core.Shutdown();
    } else {
      auto fu = core.AsyncStart();

      std::this_thread::sleep_for(std::chrono::seconds(FLAGS_running_duration));

      core.Shutdown();

      fu.wait();
    }

    global_core_ptr = nullptr;
  } catch (const std::exception& e) {
    std::cout << "AimRT run with exception and exit. " << e.what()
              << std::endl;
    return -1;
  }

  std::cout << "AimRT exit." << std::endl;

  return 0;
}
