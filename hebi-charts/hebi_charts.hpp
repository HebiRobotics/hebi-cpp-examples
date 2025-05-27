#pragma once
#include <memory>
#include <iostream>

#ifdef WIN32
#include <Windows.h>
#else
#include <dlfcn.h>
#endif


namespace hebi {
namespace charts {

// ==== Forward Declarations ====
typedef struct NativeChart_* NativeChartPtr;
typedef struct NativeChart3d_* NativeChart3dPtr;
typedef struct NativeChart3dTriad_* NativeChart3dTriadPtr;
typedef struct NativeChartAxis_* NativeChartAxisPtr;
typedef struct NativeChartDataset_* NativeChartDatasetPtr;
class JavaFxDebugUtil;
class Chart;
class Chart3d;
class Chart3dTriad;
class Axis;
class Dataset;
class ChartFramework;

enum class HebiChartTheme {
  PrimerLight,
  PrimerDark,
  NordLight,
  NordDark,
  CupertinoLight,
  CupertinoDark,
  Dracula
};


struct DynamicLookup {
  public:
    static DynamicLookup &instance() {
    #ifdef WIN32
      static DynamicLookup lib("hebi_charts.dll");
    #elif __APPLE__
      static DynamicLookup lib("libhebi_charts.dylib");
    #else
      static DynamicLookup lib("libhebi_charts.so");
    #endif
      return lib;
    }

    template<typename FuncType, typename FallbackFunc> FuncType getFunc(const char *func, FallbackFunc fallback);
    DynamicLookup(const DynamicLookup &) = delete;
    DynamicLookup &operator = (const DynamicLookup &) = delete;

  private:
    explicit DynamicLookup(const std::string &path);
    void *lib_{};
    std::string path{};
  };

// ==== C++ API Wrappers ====
class JavaFxDebugUtil {
public:
  static void setImplicitExit(bool value);
  static void exit();
  static void startupAsync();
  static void waitForNextTick();
  static void printThreadInfo(const char* name);
  static void runLater(void* /* UserCallbackFunction */ func, void* userData);
  static void runOnFx(void* /* UserCallbackFunction */ func, void* userData);
};

class Chart {
public:
  static std::shared_ptr<Chart> create();
  void show();
  void setTitle(const char* title);
  std::shared_ptr<Axis> getAxisX();
  std::shared_ptr<Axis> getAxisY();
  std::shared_ptr<Dataset> addLine(const char* name, const double* x, const double* y, int length);
  std::shared_ptr<Dataset> addBars(const char* name, const double* x, const double* y, int length);
  ~Chart();
  explicit Chart(NativeChartPtr cPointer) : ptr(cPointer) {}
private:
  operator NativeChartPtr() const { return ptr; }
  NativeChartPtr ptr;
};

class Chart3d {
public:
  static std::shared_ptr<Chart3d> create();
  void show();
  void setTitle(const char* title);
  std::shared_ptr<Chart3dTriad> addTriad(double length);
  ~Chart3d();
  explicit Chart3d(NativeChart3dPtr cPointer) : ptr(cPointer) {}
private:
  operator NativeChart3dPtr() const { return ptr; }
  NativeChart3dPtr ptr;
};

class Chart3dTriad {
public:
  void setOrientation(double qw, double qx, double qy, double qz);
  void setTranslation(double x, double y, double z);
  ~Chart3dTriad();
  explicit Chart3dTriad(NativeChart3dTriadPtr cPointer) : ptr(cPointer) {}
private:
  operator NativeChart3dTriadPtr() const { return ptr; }
  NativeChart3dTriadPtr ptr;
};

class Axis {
public:
  void setName(const char* name);
  void setUnit(const char* unit);
  void setLimits(double min, double max);
  explicit Axis(NativeChartAxisPtr cPointer) : ptr(cPointer) {}
  operator NativeChartAxisPtr() const { return ptr; }
  ~Axis();
private:
  NativeChartAxisPtr ptr;
};

class Dataset {
public:
  void setName(const char* name);
  void setData(const double* x, const double* y, int length);
  void addDataPoint(double x, double y);
  void addStyleClass(const char* style);
  ~Dataset();
  explicit Dataset(NativeChartDatasetPtr cPointer) : ptr(cPointer) {}
private:
  operator NativeChartDatasetPtr() const { return ptr; }
  NativeChartDatasetPtr ptr;
};

class ChartFramework {
public:
  static void getVersion(int* major, int* minor, int* patch, int* build);
  static bool setTheme(HebiChartTheme theme);
  static void setAutoCloseStages(bool autoClose);
  static void setWaitForAsyncCallFinish(bool emulateSyncCalls);
  static void waitUntilStagesClosed();
};

// ==== Implementations ====
// JavaFxDebugUtil
inline void JavaFxDebugUtil::setImplicitExit(bool value) {
  static auto hebiChartJavaFxDebugUtilSetImplicitExit = DynamicLookup::instance().getFunc<void(*)(bool)>("hebiChartJavaFxDebugUtilSetImplicitExit", [](bool) -> void {  });
  hebiChartJavaFxDebugUtilSetImplicitExit(value);
}
inline void JavaFxDebugUtil::exit() {
  static auto hebiChartJavaFxDebugUtilExit = DynamicLookup::instance().getFunc<void(*)()>("hebiChartJavaFxDebugUtilExit", []() -> void {  });
  hebiChartJavaFxDebugUtilExit();
}
inline void JavaFxDebugUtil::startupAsync() {
  static auto hebiChartJavaFxDebugUtilStartupAsync = DynamicLookup::instance().getFunc<void(*)()>("hebiChartJavaFxDebugUtilStartupAsync", []() -> void {  });
  hebiChartJavaFxDebugUtilStartupAsync();
}
inline void JavaFxDebugUtil::waitForNextTick() {
  static auto hebiChartJavaFxDebugUtilWaitForNextTick = DynamicLookup::instance().getFunc<void(*)()>("hebiChartJavaFxDebugUtilWaitForNextTick", []() -> void {  });
  hebiChartJavaFxDebugUtilWaitForNextTick();
}
inline void JavaFxDebugUtil::printThreadInfo(const char* name) {
  static auto hebiChartJavaFxDebugUtilPrintThreadInfo = DynamicLookup::instance().getFunc<void(*)(const char*)>("hebiChartJavaFxDebugUtilPrintThreadInfo", [](const char*) -> void {  });
  hebiChartJavaFxDebugUtilPrintThreadInfo(name);
}
inline void JavaFxDebugUtil::runLater(void* /* UserCallbackFunction */ func, void* userData) {
  static auto hebiChartJavaFxDebugUtilRunLater = DynamicLookup::instance().getFunc<void(*)(void* /* UserCallbackFunction */, void*)>("hebiChartJavaFxDebugUtilRunLater", [](void* /* UserCallbackFunction */, void*) -> void {  });
  hebiChartJavaFxDebugUtilRunLater(func, userData);
}
inline void JavaFxDebugUtil::runOnFx(void* /* UserCallbackFunction */ func, void* userData) {
  static auto hebiChartJavaFxDebugUtilRunOnFx = DynamicLookup::instance().getFunc<void(*)(void* /* UserCallbackFunction */, void*)>("hebiChartJavaFxDebugUtilRunOnFx", [](void* /* UserCallbackFunction */, void*) -> void {  });
  hebiChartJavaFxDebugUtilRunOnFx(func, userData);
}

// Chart
inline std::shared_ptr<Chart> Chart::create() {
  static auto hebiChartCreate = DynamicLookup::instance().getFunc<NativeChartPtr(*)()>("hebiChartCreate", []() -> NativeChartPtr { return nullptr; });
  return std::make_shared<Chart>(hebiChartCreate());
}
inline void Chart::show() {
  static auto hebiChartShow = DynamicLookup::instance().getFunc<void(*)(NativeChartPtr)>("hebiChartShow", [](NativeChartPtr) -> void {  });
  hebiChartShow(this->ptr);
}
inline void Chart::setTitle(const char* title) {
  static auto hebiChartSetTitle = DynamicLookup::instance().getFunc<void(*)(NativeChartPtr, const char*)>("hebiChartSetTitle", [](NativeChartPtr, const char*) -> void {  });
  hebiChartSetTitle(this->ptr, title);
}
inline std::shared_ptr<Axis> Chart::getAxisX() {
  static auto hebiChartGetAxisX = DynamicLookup::instance().getFunc<NativeChartAxisPtr(*)(NativeChartPtr)>("hebiChartGetAxisX", [](NativeChartPtr) -> NativeChartAxisPtr { return nullptr; });
  return std::make_shared<Axis>(hebiChartGetAxisX(this->ptr));
}
inline std::shared_ptr<Axis> Chart::getAxisY() {
  static auto hebiChartGetAxisY = DynamicLookup::instance().getFunc<NativeChartAxisPtr(*)(NativeChartPtr)>("hebiChartGetAxisY", [](NativeChartPtr) -> NativeChartAxisPtr { return nullptr; });
  return std::make_shared<Axis>(hebiChartGetAxisY(this->ptr));
}
inline std::shared_ptr<Dataset> Chart::addLine(const char* name, const double* x, const double* y, int length) {
  static auto hebiChartAddLine = DynamicLookup::instance().getFunc<NativeChartDatasetPtr(*)(NativeChartPtr, const char*, const double*, const double*, int)>("hebiChartAddLine", [](NativeChartPtr, const char*, const double*, const double*, int) -> NativeChartDatasetPtr { return nullptr; });
  return std::make_shared<Dataset>(hebiChartAddLine(this->ptr, name, x, y, length));
}
inline std::shared_ptr<Dataset> Chart::addBars(const char* name, const double* x, const double* y, int length) {
  static auto hebiChartAddBars = DynamicLookup::instance().getFunc<NativeChartDatasetPtr(*)(NativeChartPtr, const char*, const double*, const double*, int)>("hebiChartAddBars", [](NativeChartPtr, const char*, const double*, const double*, int) -> NativeChartDatasetPtr { return nullptr; });
  return std::make_shared<Dataset>(hebiChartAddBars(this->ptr, name, x, y, length));
}
inline Chart::~Chart() {
  static auto hebiChartRelease = DynamicLookup::instance().getFunc<void(*)(NativeChartPtr)>("hebiChartRelease", [](NativeChartPtr) -> void{});
  hebiChartRelease(this->ptr);
}

// Chart3d
inline std::shared_ptr<Chart3d> Chart3d::create() {
  static auto hebiChart3dCreate = DynamicLookup::instance().getFunc<NativeChart3dPtr(*)()>("hebiChart3dCreate", []() -> NativeChart3dPtr { return nullptr; });
  return std::make_shared<Chart3d>(hebiChart3dCreate());
}
inline void Chart3d::show() {
  static auto hebiChart3dShow = DynamicLookup::instance().getFunc<void(*)(NativeChart3dPtr)>("hebiChart3dShow", [](NativeChart3dPtr) -> void {  });
  hebiChart3dShow(this->ptr);
}
inline void Chart3d::setTitle(const char* title) {
  static auto hebiChart3dSetTitle = DynamicLookup::instance().getFunc<void(*)(NativeChart3dPtr, const char*)>("hebiChart3dSetTitle", [](NativeChart3dPtr, const char*) -> void {  });
  hebiChart3dSetTitle(this->ptr, title);
}
inline std::shared_ptr<Chart3dTriad> Chart3d::addTriad(double length) {
  static auto hebiChart3dAddTriad = DynamicLookup::instance().getFunc<NativeChart3dTriadPtr(*)(NativeChart3dPtr, double)>("hebiChart3dAddTriad", [](NativeChart3dPtr, double) -> NativeChart3dTriadPtr { return nullptr; });
  return std::make_shared<Chart3dTriad>(hebiChart3dAddTriad(this->ptr, length));
}
inline Chart3d::~Chart3d() {
  static auto hebiChart3dRelease = DynamicLookup::instance().getFunc<void(*)(NativeChart3dPtr)>("hebiChart3dRelease", [](NativeChart3dPtr) -> void{});
  hebiChart3dRelease(this->ptr);
}

// Chart3dTriad
inline void Chart3dTriad::setOrientation(double qw, double qx, double qy, double qz) {
  static auto hebiChart3dTriadSetOrientation = DynamicLookup::instance().getFunc<void(*)(NativeChart3dTriadPtr, double, double, double, double)>("hebiChart3dTriadSetOrientation", [](NativeChart3dTriadPtr, double, double, double, double) -> void {  });
  hebiChart3dTriadSetOrientation(this->ptr, qw, qx, qy, qz);
}
inline void Chart3dTriad::setTranslation(double x, double y, double z) {
  static auto hebiChart3dTriadSetTranslation = DynamicLookup::instance().getFunc<void(*)(NativeChart3dTriadPtr, double, double, double)>("hebiChart3dTriadSetTranslation", [](NativeChart3dTriadPtr, double, double, double) -> void {  });
  hebiChart3dTriadSetTranslation(this->ptr, x, y, z);
}
inline Chart3dTriad::~Chart3dTriad() {
  static auto hebiChart3dTriadRelease = DynamicLookup::instance().getFunc<void(*)(NativeChart3dTriadPtr)>("hebiChart3dTriadRelease", [](NativeChart3dTriadPtr) -> void{});
  hebiChart3dTriadRelease(this->ptr);
}

// Axis
inline void Axis::setName(const char* name) {
  static auto hebiChartAxisSetName = DynamicLookup::instance().getFunc<void(*)(NativeChartAxisPtr, const char*)>("hebiChartAxisSetName", [](NativeChartAxisPtr, const char*) -> void {  });
  hebiChartAxisSetName(this->ptr, name);
}
inline void Axis::setUnit(const char* unit) {
  static auto hebiChartAxisSetUnit = DynamicLookup::instance().getFunc<void(*)(NativeChartAxisPtr, const char*)>("hebiChartAxisSetUnit", [](NativeChartAxisPtr, const char*) -> void {  });
  hebiChartAxisSetUnit(this->ptr, unit);
}
inline void Axis::setLimits(double min, double max) {
  static auto hebiChartAxisSetLimits = DynamicLookup::instance().getFunc<void(*)(NativeChartAxisPtr, double, double)>("hebiChartAxisSetLimits", [](NativeChartAxisPtr, double, double) -> void {  });
  hebiChartAxisSetLimits(this->ptr, min, max);
}
inline Axis::~Axis() {
  static auto hebiChartAxisRelease = DynamicLookup::instance().getFunc<void(*)(NativeChartAxisPtr)>("hebiChartAxisRelease", [](NativeChartAxisPtr) -> void{});
  hebiChartAxisRelease(this->ptr);
}

// Dataset
inline void Dataset::setName(const char* name) {
  static auto hebiChartDatasetSetName = DynamicLookup::instance().getFunc<void(*)(NativeChartDatasetPtr, const char*)>("hebiChartDatasetSetName", [](NativeChartDatasetPtr, const char*) -> void {  });
  hebiChartDatasetSetName(this->ptr, name);
}
inline void Dataset::setData(const double* x, const double* y, int length) {
  static auto hebiChartDatasetSetData = DynamicLookup::instance().getFunc<void(*)(NativeChartDatasetPtr, const double*, const double*, int)>("hebiChartDatasetSetData", [](NativeChartDatasetPtr, const double*, const double*, int) -> void {  });
  hebiChartDatasetSetData(this->ptr, x, y, length);
}
inline void Dataset::addDataPoint(double x, double y) {
  static auto hebiChartDatasetAddDataPoint = DynamicLookup::instance().getFunc<void(*)(NativeChartDatasetPtr, double, double)>("hebiChartDatasetAddDataPoint", [](NativeChartDatasetPtr, double, double) -> void {  });
  hebiChartDatasetAddDataPoint(this->ptr, x, y);
}
inline void Dataset::addStyleClass(const char* style) {
  static auto hebiChartDatasetAddStyleClass = DynamicLookup::instance().getFunc<void(*)(NativeChartDatasetPtr, const char*)>("hebiChartDatasetAddStyleClass", [](NativeChartDatasetPtr, const char*) -> void {  });
  hebiChartDatasetAddStyleClass(this->ptr, style);
}
inline Dataset::~Dataset() {
  static auto hebiChartDatasetRelease = DynamicLookup::instance().getFunc<void(*)(NativeChartDatasetPtr)>("hebiChartDatasetRelease", [](NativeChartDatasetPtr) -> void{});
  hebiChartDatasetRelease(this->ptr);
}

// ChartFramework
inline void ChartFramework::getVersion(int* major, int* minor, int* patch, int* build) {
  static auto hebiChartFrameworkGetVersion = DynamicLookup::instance().getFunc<void(*)(int*, int*, int*, int*)>("hebiChartFrameworkGetVersion", [](int*, int*, int*, int*) -> void {  });
  hebiChartFrameworkGetVersion(major, minor, patch, build);
}
inline bool ChartFramework::setTheme(HebiChartTheme theme) {
  static auto hebiChartFrameworkSetTheme = DynamicLookup::instance().getFunc<bool(*)(HebiChartTheme)>("hebiChartFrameworkSetTheme", [](HebiChartTheme) -> bool { return 0; });
  return hebiChartFrameworkSetTheme(theme);
}
inline void ChartFramework::setAutoCloseStages(bool autoClose) {
  static auto hebiChartFrameworkSetAutoCloseStages = DynamicLookup::instance().getFunc<void(*)(bool)>("hebiChartFrameworkSetAutoCloseStages", [](bool) -> void {  });
  hebiChartFrameworkSetAutoCloseStages(autoClose);
}
inline void ChartFramework::setWaitForAsyncCallFinish(bool emulateSyncCalls) {
  static auto hebiChartFrameworkSetWaitForAsyncCallFinish = DynamicLookup::instance().getFunc<void(*)(bool)>("hebiChartFrameworkSetWaitForAsyncCallFinish", [](bool) -> void {  });
  hebiChartFrameworkSetWaitForAsyncCallFinish(emulateSyncCalls);
}
inline void ChartFramework::waitUntilStagesClosed() {
  static auto hebiChartFrameworkWaitUntilStagesClosed = DynamicLookup::instance().getFunc<void(*)()>("hebiChartFrameworkWaitUntilStagesClosed", []() -> void {  });
  hebiChartFrameworkWaitUntilStagesClosed();
}

// ==== Cocoa utilities for supporting macOS ====
typedef int (*hebiCharts_main_func_t)(int argc, char** argv);
inline int hebiChartsRunApplication(hebiCharts_main_func_t callback, int argc, char** argv) {
    static auto hebiChartsRunApplicationFn = DynamicLookup::instance().getFunc<int(*)(hebiCharts_main_func_t, int, char**)>("hebiChartsRunApplication", [](hebiCharts_main_func_t, int, char**) -> int { return -1; });
    return hebiChartsRunApplicationFn(callback, argc, argv);
}

template<typename FuncType, typename FallbackFunc> FuncType DynamicLookup::getFunc(const char *func, FallbackFunc fallback) {
  if (lib_) {
  #ifdef WIN32
    FARPROC symbol = ::GetProcAddress((HMODULE) lib_, func);
  #else
    void* symbol = dlsym(lib_, func);
  #endif
    if (symbol) {
      return reinterpret_cast<FuncType>(symbol);
    } else {
      std::cerr << "Library \"" << path << "\" is missing symbol \"" << func <<
        "\"! Please choose a compatible version.\n";
    }
  } else {
    // TODO: show error for each call or just once?
    // std::cerr << "Library not found! Include \"" << path << "\" in library search path to use \"" << func << "\".\n";
  }
  return fallback;
}

DynamicLookup:: DynamicLookup(const std::string &path) {
  this->path = path;
#ifdef WIN32
  // convert to wide string to support non-ascii paths
  int size_needed = MultiByteToWideChar(CP_UTF8, 0, &path[0], (int)path.size(), NULL, 0);
  std::wstring wstrTo(size_needed, 0);
  MultiByteToWideChar(CP_UTF8, 0, &path[0], (int)path.size(), &wstrTo[0], size_needed);
  lib_ = ::LoadLibraryW(wstrTo.c_str());
#else
  lib_ = dlopen(path.c_str(), RTLD_LAZY);
#endif
  if(!lib_) {
    std::cerr << "Library not found! Include \""<< path <<"\" in library search path.\n";
  }
}

} // namespace hebi
} // namespace charts
