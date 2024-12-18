#pragma once
#include <memory>

#ifdef WIN32
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

namespace hebi {
namespace plotting {

// ==== C++ API Wrappers ====

typedef struct NativeChart_* NativeChartPtr;
typedef struct NativeChartDataset_* NativeChartDatasetPtr;

struct PlotLibWrapper {
  inline void* getFunc(void* lib, const char* func)
  {
    return GetProcAddress((HINSTANCE)lib, func);
  }
public:
  static PlotLibWrapper& instance()
  {
    static PlotLibWrapper lib("libhebi_java.dll");
    return lib;
  }

  PlotLibWrapper(const PlotLibWrapper&) = delete;
  PlotLibWrapper& operator = (const PlotLibWrapper&) = delete;
private:
  PlotLibWrapper(const std::string& path)
  {
    lib_ = LoadLibraryA(path.c_str());
    if (!lib_)
    {
      hebiChartCreate = []() -> NativeChartPtr {
        std::cerr << "Plotting library not found! Include \"hebi_plot\" in library search path.\n";
        return nullptr;
      };
      hebiChartShow = [](NativeChartPtr) {};
      hebiChartSetTitle = [](NativeChartPtr, const char*) {};
      hebiChartAddLine = [](NativeChartPtr, const char*, const double*, const double*, int) -> NativeChartDatasetPtr { return nullptr; };
      hebiChartRelease = [](NativeChartPtr) {};

      hebiFxRuntimeSetTheme = [](const char* theme) -> void* {
        std::cerr << "Plotting library not found! Include \"hebi_plot\" in library search path.\n";
        return nullptr;
      };
      hebiFxRuntimeSetImplicitExit = [](bool value) {};
      hebiFxRuntimeExit = []() {};
      hebiFxRuntimeWaitUntilStagesClosed = []() {};
      
      hebiChartDatasetSetName = [](NativeChartDatasetPtr obj, const char* name) {};
      hebiChartDatasetSetData = [](NativeChartDatasetPtr obj, const double* x, const double* y, int length) {};
      hebiChartDatasetAddStyleClass = [](NativeChartDatasetPtr obj, const char* style) {};
      hebiChartDatasetRelease = [](NativeChartDatasetPtr obj) {};
    }
    else
    {
      hebiChartCreate = (NativeChartPtr(*)())getFunc(lib_, "hebiChartCreate");
      hebiChartShow = (void(*)(NativeChartPtr))getFunc(lib_, "hebiChartShow");
      hebiChartSetTitle = (void(*)(NativeChartPtr, const char*))getFunc(lib_, "hebiChartSetTitle");
      hebiChartAddLine = (NativeChartDatasetPtr(*)(NativeChartPtr, const char*, const double*, const double*, int))getFunc(lib_, "hebiChartAddLine");
      hebiChartRelease = (void(*)(NativeChartPtr))getFunc(lib_, "hebiChartRelease");
      hebiFxRuntimeSetTheme = (void* (*)(const char* theme))getFunc(lib_, "hebiFxRuntimeSetTheme");
      hebiFxRuntimeSetImplicitExit = (void(*)(bool value))getFunc(lib_, "hebiFxRuntimeSetImplicitExit");
      hebiFxRuntimeExit = (void(*)())getFunc(lib_, "hebiFxRuntimeExit");
      hebiFxRuntimeWaitUntilStagesClosed = (void(*)())getFunc(lib_, "hebiFxRuntimeWaitUntilStagesClosed");
      hebiChartDatasetSetName = (void(*)(NativeChartDatasetPtr obj, const char* name))getFunc(lib_, "hebiChartDatasetSetName");
      hebiChartDatasetSetData = (void(*)(NativeChartDatasetPtr obj, const double* x, const double* y, int length))getFunc(lib_, "hebiChartDatasetSetData");
      hebiChartDatasetAddStyleClass = (void(*)(NativeChartDatasetPtr obj, const char* style))getFunc(lib_, "hebiChartDatasetAddStyleClass");
      hebiChartDatasetRelease = (void(*)(NativeChartDatasetPtr obj))getFunc(lib_, "hebiChartDatasetRelease");
    }
  }

public:

  NativeChartPtr(* hebiChartCreate)() = nullptr;
  void(*hebiChartShow)(NativeChartPtr) = nullptr;
  void(*hebiChartSetTitle)(NativeChartPtr, const char*) = nullptr;
  NativeChartDatasetPtr(*hebiChartAddLine)(NativeChartPtr, const char*, const double*, const double*, int) = nullptr;
  void(*hebiChartRelease)(NativeChartPtr) = nullptr;

  void* (*hebiFxRuntimeSetTheme)(const char* theme) = nullptr;
  void(*hebiFxRuntimeSetImplicitExit)(bool value) = nullptr;
  void(*hebiFxRuntimeExit)() = nullptr;
  void(*hebiFxRuntimeWaitUntilStagesClosed)() = nullptr;

  void(*hebiChartDatasetSetName)(NativeChartDatasetPtr obj, const char* name) = nullptr;
  void(*hebiChartDatasetSetData)(NativeChartDatasetPtr obj, const double* x, const double* y, int length) = nullptr;
  void(*hebiChartDatasetAddStyleClass)(NativeChartDatasetPtr obj, const char* style) = nullptr;
  void(*hebiChartDatasetRelease)(NativeChartDatasetPtr obj) = nullptr;

  ~PlotLibWrapper()
  {
  }

private:
  void* lib_{};
};

class Dataset {
public:
  void setName(const char* name);
  void setData(const double* x, const double* y, int length);
  void addStyleClass(const char* style);
  explicit Dataset(NativeChartDatasetPtr cPointer) : lib(PlotLibWrapper::instance()), ptr(cPointer) {}
  operator NativeChartDatasetPtr() const { return ptr; }
  ~Dataset() { lib.hebiChartDatasetRelease(this->ptr); }
private:
  PlotLibWrapper& lib;
  NativeChartDatasetPtr ptr;
};

class Chart {
public:
  static std::shared_ptr<Chart> create();
  void show();
  void setTitle(const char* title);
  std::shared_ptr<Dataset> addLine(const char* name, const double* x, const double* y, int length);
  explicit Chart(NativeChartPtr cPointer) : lib(PlotLibWrapper::instance()), ptr(cPointer) {}
  operator NativeChartPtr() const { return ptr; }
  ~Chart() { lib.hebiChartRelease(this->ptr); }
private:
  PlotLibWrapper& lib;
  NativeChartPtr ptr;
};

class FxRuntime {
public:
  FxRuntime() {}
  static void* /* HebiStatusCode */ setTheme(const char* theme);
  static void setImplicitExit(bool value);
  static void exit();
  static void waitUntilStagesClosed();
private:
};

// ==== Implementations ====
// Chart
inline std::shared_ptr<Chart> Chart::create() {
  return std::make_shared<Chart>(PlotLibWrapper::instance().hebiChartCreate());
}
inline void Chart::show() {
  lib.hebiChartShow(this->ptr);
}
inline void Chart::setTitle(const char* title) {
  lib.hebiChartSetTitle(this->ptr, title);
}
inline std::shared_ptr<Dataset> Chart::addLine(const char* name, const double* x, const double* y, int length) {
  return std::make_shared<Dataset>(lib.hebiChartAddLine(this->ptr, name, x, y, length));
}

// FxRuntime
inline void* /* HebiStatusCode */ FxRuntime::setTheme(const char* theme) {
  return PlotLibWrapper::instance().hebiFxRuntimeSetTheme(theme);
}
inline void FxRuntime::setImplicitExit(bool value) {
  PlotLibWrapper::instance().hebiFxRuntimeSetImplicitExit(value);
}
inline void FxRuntime::exit() {
  PlotLibWrapper::instance().hebiFxRuntimeExit();
}
inline void FxRuntime::waitUntilStagesClosed() {
  PlotLibWrapper::instance().hebiFxRuntimeWaitUntilStagesClosed();
}

// Dataset
inline void Dataset::setName(const char* name) {
  lib.hebiChartDatasetSetName(this->ptr, name);
}
inline void Dataset::setData(const double* x, const double* y, int length) {
  lib.hebiChartDatasetSetData(this->ptr, x, y, length);
}
inline void Dataset::addStyleClass(const char* style) {
  lib.hebiChartDatasetAddStyleClass(this->ptr, style);
}

} // namespace plotting
} // namespace hebi
