// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/logger/crash_signal_handling.h"

#include <signal.h>
#include <ucontext.h>
#include <unistd.h>
#include <string_view>

#include "backward.hpp"
#include "util/log_util.h"

namespace aimrt::runtime::core::logger {

std::vector<int> DefaultSignals() {
  return backward::SignalHandling::make_default_signals();
}

void CrashSignalHandling::FreeDeleter::operator()(char* p) const noexcept {}

CrashSignalHandling::CrashSignalHandling() { InstallHandlers(DefaultSignals()); }

void CrashSignalHandling::HandleSignal(int, siginfo_t* info, void* _ctx) {
  ucontext_t* uctx = static_cast<ucontext_t*>(_ctx);

  backward::StackTrace st;
  void* error_addr = nullptr;

#ifdef REG_RIP  // x86_64
  error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.gregs[REG_RIP]);
#elif defined(REG_EIP)  // x86_32
  error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.gregs[REG_EIP]);
#elif defined(__arm__)
  error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.arm_pc);
#elif defined(__aarch64__)
  #if defined(__APPLE__)
  error_addr = reinterpret_cast<void*>(uctx->uc_mcontext->__ss.__pc);
  #else
  error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.pc);
  #endif
#elif defined(__mips__)
  error_addr = reinterpret_cast<void*>(
      reinterpret_cast<struct sigcontext*>(&uctx->uc_mcontext)->sc_pc);
#elif defined(__ppc__) || defined(__powerpc) || defined(__powerpc__) || defined(__POWERPC__)
  error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.regs->nip);
#elif defined(__riscv)
  error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.__gregs[REG_PC]);
#elif defined(__s390x__)
  error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.psw.addr);
#elif defined(__APPLE__) && defined(__x86_64__)
  error_addr = reinterpret_cast<void*>(uctx->uc_mcontext->__ss.__rip);
#elif defined(__APPLE__)
  error_addr = reinterpret_cast<void*>(uctx->uc_mcontext->__ss.__eip);
#endif

  if (error_addr) {
    st.load_from(error_addr, 32, reinterpret_cast<void*>(uctx), info->si_addr);
  } else {
    st.load_here(32, reinterpret_cast<void*>(uctx), info->si_addr);
  }
  backward::Printer printer;
  printer.address = true;
  std::stringstream ss;
  printer.print(st, ss);
  AIMRT_ERROR("CRASH SIGNAL HANDLING SIGNAL: {}", info->si_signo);
  AIMRT_ERROR("CRASH SIGNAL HANDLING STACKTRACE: \n {}", ss.str());

#if _XOPEN_SOURCE >= 700 || _POSIX_C_SOURCE >= 200809L
  psiginfo(info, nullptr);
#else
  (void)info;
#endif
}

#ifdef __GNUC__
__attribute__((noreturn))
#endif
void
CrashSignalHandling::SigHandler(int signo, siginfo_t* info, void* _ctx) {
  HandleSignal(signo, info, _ctx);

  // Forward the signal (SA_RESETHAND makes it use default on re-raise).
  raise(info->si_signo);
  _exit(EXIT_FAILURE);
}

void CrashSignalHandling::EnsureAltStack() {
  const size_t stack_size = 1024 * 1024 * 8;
  alt_stack_.reset(static_cast<char*>(std::malloc(stack_size)));
  if (!alt_stack_) {
    loaded_ = false;
    return;
  }

  stack_t ss;
  ss.ss_sp = alt_stack_.get();
  ss.ss_size = stack_size;
  ss.ss_flags = 0;
  if (sigaltstack(&ss, nullptr) < 0) {
    loaded_ = false;
  }
}

void CrashSignalHandling::InstallHandlers(const std::vector<int>& signals) {
  loaded_ = true;
  EnsureAltStack();

  for (int sig : signals) {
    struct sigaction action;
    std::memset(&action, 0, sizeof action);
    action.sa_flags = static_cast<int>(SA_SIGINFO | SA_ONSTACK | SA_NODEFER | SA_RESETHAND);
    sigfillset(&action.sa_mask);
    sigdelset(&action.sa_mask, sig);
    action.sa_sigaction = &SigHandler;

    if (sigaction(sig, &action, nullptr) < 0) {
      loaded_ = false;
    }
  }
}

}  // namespace aimrt::runtime::core::logger
