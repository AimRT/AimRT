// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

#if defined(_WIN32)
  #include <windows.h>
#else
  #include <dlfcn.h>
#endif

namespace aimrt::common::util {

class DynamicLib {
 public:
#if defined(_WIN32)
  typedef FARPROC SymbolType;
  typedef HINSTANCE DynlibHandle;
#else
  typedef void* SymbolType;
  typedef void* DynlibHandle;
#endif

 public:
  DynamicLib() = default;
  ~DynamicLib() { Unload(); }

  DynamicLib(const DynamicLib&) = delete;
  DynamicLib& operator=(const DynamicLib&) = delete;

  bool Load(const std::string& libname) {
    Unload();
    libname_ = libname;

#if defined(_WIN32)
    handle_ = LoadLibraryEx(libname_.c_str(), NULL, 0);

    if (nullptr == handle_) return false;

    TCHAR buf[MAX_PATH];
    DWORD re = GetModuleFileName(handle_, buf, MAX_PATH);
    if (re != 0) lib_full_path_ = std::string(buf);
#else
    handle_ = dlopen(libname_.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);

    if (nullptr == handle_) return false;

    char buf[1024];
    dlinfo(handle_, RTLD_DI_ORIGIN, &buf);
    lib_full_path_ = std::filesystem::canonical(std::filesystem::path(buf) / libname_).string();
#endif

    return true;
  }

  void Unload() {
    if (nullptr == handle_) return;

    bool ret = false;
#if defined(_WIN32)
    ret = FreeLibrary(handle_);
#else
    ret = (0 == dlclose(handle_));
#endif

    if (!ret) {
      return;
    }

    handle_ = nullptr;
  }

  bool IsLoaded() const { return nullptr == handle_; }

  SymbolType GetSymbol(const std::string& symbol_name) {
    if (nullptr == handle_)
      throw std::runtime_error("DynamicLib does not load any lib.");

#if defined(_WIN32)
    return GetProcAddress(handle_, symbol_name.c_str());
#else
    return dlsym(handle_, symbol_name.c_str());
#endif
  }

  const std::string& GetLibName() const { return libname_; }

  const std::string& GetLibFullPath() const { return lib_full_path_; }

  static std::string GetErr() {
#if defined(_WIN32)
    LPTSTR lp_msg_buf;
    FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                  NULL,
                  GetLastError(),
                  MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                  (LPTSTR)&lp_msg_buf,
                  0,
                  NULL);
    std::string ret(lp_msg_buf);
    LocalFree(lp_msg_buf);
    return ret;
#else
    return std::string(dlerror());
#endif
  }

 private:
  DynlibHandle handle_ = nullptr;
  std::string libname_;
  std::string lib_full_path_;
};

}  // namespace aimrt::common::util
