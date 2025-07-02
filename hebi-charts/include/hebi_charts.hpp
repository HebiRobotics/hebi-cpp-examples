#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

#ifdef WIN32
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

namespace hebi {
namespace charts {

enum class Color {
  Default,
  Primary,
  PrimaryMuted,
  Red,
  Green,
  Blue,
  Yellow,
  Magenta
};

enum class LineStyle {
  Default,
  Solid,
  Dashed,
  Points
};

enum class Theme {
  PrimerLight,
  PrimerDark,
  NordLight,
  NordDark,
  CupertinoLight,
  CupertinoDark,
  Dracula
};

// ==== Forward Declarations ====
class Chart;
class Chart3d;
class Chart3dModel;
class Chart3dTriad;
class Axis;
class Dataset;

namespace {

using NativeChartPtr = struct NativeChart_*;
using NativeChart3dPtr = struct NativeChart3d_*;
using NativeChart3dModelPtr = struct NativeChart3dModel_*;
using NativeChart3dTriadPtr = struct NativeChart3dTriad_*;
using NativeChartAxisPtr = struct NativeChartAxis_*;
using NativeChartDatasetPtr = struct NativeChartDataset_*;

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

  bool isLoaded() const { return lib_ != nullptr; };

  template<typename FuncType> FuncType getFunc(const char *func) const;
  DynamicLookup(const DynamicLookup &) = delete;
  DynamicLookup &operator = (const DynamicLookup &) = delete;

private:
  explicit DynamicLookup(const std::string &path);
  void* lib_{};
  std::string path_{};
};

} // namespace


// ==== C++ API Wrappers ====
class Chart {
public:
  Chart();
  void show();
  void setTitle(const std::string& title);
  void setTitle(const char* title);
  Axis getAxisX();
  Axis getAxisY();
  Dataset addLine(const std::string& name, const std::vector<double>& x, const std::vector<double>& y);
  Dataset addLine(const char* name, const double* x, const double* y, size_t length);
  Dataset addBars(const std::string& name, const std::vector<double>& x, const std::vector<double>& y);
  Dataset addBars(const char* name, const double* x, const double* y, size_t length);
  Chart(Chart&& from) noexcept;
  Chart& operator=(Chart&& from) noexcept;
  ~Chart();
private:
  void cleanup();
  explicit Chart(NativeChartPtr cPointer) : ptr_(cPointer) {}
  NativeChartPtr ptr_{};
};

class Chart3d {
public:
  Chart3d();
  void show();
  void setTitle(const std::string& title);
  void setTitle(const char* title);
  Chart3dModel addHrdf(const std::string& filePath);
  Chart3dModel addHrdf(const char* filePath);
  Chart3dTriad addTriad(double length);
  Chart3d(Chart3d&& from) noexcept;
  Chart3d& operator=(Chart3d&& from) noexcept;
  ~Chart3d();
private:
  void cleanup();
  explicit Chart3d(NativeChart3dPtr cPointer) : ptr_(cPointer) {}
  NativeChart3dPtr ptr_{};
};

class Chart3dModel {
  friend class Chart3d;
public:
  size_t getNumJoints();
  void setPositions(const std::vector<double>& positions);
  void setPositions(const double* positions, size_t length);
  void setOrientation(double qw, double qx, double qy, double qz);
  void setTranslation(double x, double y, double z);
  void setTransform4x4(const std::vector<double>& matrix);
  void setTransform4x4(const double* matrix, size_t length);
  Chart3dModel(Chart3dModel&& from) noexcept;
  Chart3dModel& operator=(Chart3dModel&& from) noexcept;
  ~Chart3dModel();
private:
  void cleanup();
  explicit Chart3dModel(NativeChart3dModelPtr cPointer) : ptr_(cPointer) {}
  NativeChart3dModelPtr ptr_{};
};

class Chart3dTriad {
  friend class Chart3d;
public:
  void setOrientation(double qw, double qx, double qy, double qz);
  void setTranslation(double x, double y, double z);
  void setTransform4x4(const std::vector<double>& matrix);
  void setTransform4x4(const double* matrix, size_t length);
  Chart3dTriad(Chart3dTriad&& from) noexcept;
  Chart3dTriad& operator=(Chart3dTriad&& from) noexcept;
  ~Chart3dTriad();
private:
  void cleanup();
  explicit Chart3dTriad(NativeChart3dTriadPtr cPointer) : ptr_(cPointer) {}
  NativeChart3dTriadPtr ptr_{};
};

class Axis {
  friend class Chart;
public:
  void setName(const std::string& name);
  void setName(const char* name);
  void setUnit(const std::string& unit);
  void setUnit(const char* unit);
  void setLimits(double min, double max);
  Axis(Axis&& from) noexcept;
  Axis& operator=(Axis&& from) noexcept;
  ~Axis();
private:
  void cleanup();
  explicit Axis(NativeChartAxisPtr cPointer) : ptr_(cPointer) {}
  NativeChartAxisPtr ptr_{};
};

class Dataset {
  friend class Chart;
public:
  void setName(const std::string& name);
  void setName(const char* name);
  void setData(const std::vector<double>& x, const std::vector<double>& y);
  void setData(const double* x, const double* y, size_t length);
  void addPoint(double x, double y);
  void setColor(Color color);
  void setLineStyle(LineStyle lineStyle);
  Dataset(Dataset&& from) noexcept;
  Dataset& operator=(Dataset&& from) noexcept;
  ~Dataset();
private:
  void cleanup();
  explicit Dataset(NativeChartDatasetPtr cPointer) : ptr_(cPointer) {}
  NativeChartDatasetPtr ptr_{};
};

namespace framework {

bool setTheme(Theme theme);
void setAutoCloseWindows(bool autoClose);
void waitUntilWindowsClosed();

} // namespace framework

// ==== C++ Implementations ====

// Chart
inline Chart::Chart() {
  static auto hebiChartsChartCreate = DynamicLookup::instance().getFunc<NativeChartPtr(*)()>("hebiChartsChartCreate");
  ptr_ = hebiChartsChartCreate();
}
inline void Chart::show() {
  static auto hebiChartsChartShow = DynamicLookup::instance().getFunc<void(*)(NativeChartPtr)>("hebiChartsChartShow");
  hebiChartsChartShow(ptr_);
}
inline void Chart::setTitle(const std::string& title) {
   setTitle(title.c_str());
}
inline void Chart::setTitle(const char* title) {
  static auto hebiChartsChartSetTitle = DynamicLookup::instance().getFunc<void(*)(NativeChartPtr, const char*)>("hebiChartsChartSetTitle");
  hebiChartsChartSetTitle(ptr_, title);
}
inline Axis Chart::getAxisX() {
  static auto hebiChartsChartGetAxisX = DynamicLookup::instance().getFunc<NativeChartAxisPtr(*)(NativeChartPtr)>("hebiChartsChartGetAxisX");
  return Axis(hebiChartsChartGetAxisX(ptr_));
}
inline Axis Chart::getAxisY() {
  static auto hebiChartsChartGetAxisY = DynamicLookup::instance().getFunc<NativeChartAxisPtr(*)(NativeChartPtr)>("hebiChartsChartGetAxisY");
  return Axis(hebiChartsChartGetAxisY(ptr_));
}
inline Dataset Chart::addLine(const std::string& name, const std::vector<double>& x, const std::vector<double>& y) {
  return addLine(name.c_str(), x.data(), y.data(), (std::min)(x.size(), y.size()));
}
inline Dataset Chart::addLine(const char* name, const double* x, const double* y, size_t length) {
  static auto hebiChartsChartAddLine = DynamicLookup::instance().getFunc<NativeChartDatasetPtr(*)(NativeChartPtr, const char*, const double*, const double*, size_t)>("hebiChartsChartAddLine");
  return Dataset(hebiChartsChartAddLine(ptr_, name, x, y, length));
}
inline Dataset Chart::addBars(const std::string& name, const std::vector<double>& x, const std::vector<double>& y) {
  return addBars(name.c_str(), x.data(), y.data(), (std::min)(x.size(), y.size()));
}
inline Dataset Chart::addBars(const char* name, const double* x, const double* y, size_t length) {
  static auto hebiChartsChartAddBars = DynamicLookup::instance().getFunc<NativeChartDatasetPtr(*)(NativeChartPtr, const char*, const double*, const double*, size_t)>("hebiChartsChartAddBars");
  return Dataset(hebiChartsChartAddBars(ptr_, name, x, y, length));
}
inline void Chart::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebiChartsChartRelease = DynamicLookup::instance().getFunc<void(*)(NativeChartPtr)>("hebiChartsChartRelease");
    hebiChartsChartRelease(ptr_);
  }
}
inline Chart::Chart(Chart&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Chart& Chart::operator=(Chart&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Chart::~Chart() {
  cleanup();
}

// Chart3d
inline Chart3d::Chart3d() {
  static auto hebiChartsChart3dCreate = DynamicLookup::instance().getFunc<NativeChart3dPtr(*)()>("hebiChartsChart3dCreate");
  ptr_ = hebiChartsChart3dCreate();
}
inline void Chart3d::show() {
  static auto hebiChartsChart3dShow = DynamicLookup::instance().getFunc<void(*)(NativeChart3dPtr)>("hebiChartsChart3dShow");
  hebiChartsChart3dShow(ptr_);
}
inline void Chart3d::setTitle(const std::string& title) {
   setTitle(title.c_str());
}
inline void Chart3d::setTitle(const char* title) {
  static auto hebiChartsChart3dSetTitle = DynamicLookup::instance().getFunc<void(*)(NativeChart3dPtr, const char*)>("hebiChartsChart3dSetTitle");
  hebiChartsChart3dSetTitle(ptr_, title);
}
inline Chart3dModel Chart3d::addHrdf(const std::string& filePath) {
  return addHrdf(filePath.c_str());
}
inline Chart3dModel Chart3d::addHrdf(const char* filePath) {
  static auto hebiChartsChart3dAddHrdf = DynamicLookup::instance().getFunc<NativeChart3dModelPtr(*)(NativeChart3dPtr, const char*)>("hebiChartsChart3dAddHrdf");
  return Chart3dModel(hebiChartsChart3dAddHrdf(ptr_, filePath));
}
inline Chart3dTriad Chart3d::addTriad(double length) {
  static auto hebiChartsChart3dAddTriad = DynamicLookup::instance().getFunc<NativeChart3dTriadPtr(*)(NativeChart3dPtr, double)>("hebiChartsChart3dAddTriad");
  return Chart3dTriad(hebiChartsChart3dAddTriad(ptr_, length));
}
inline void Chart3d::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebiChartsChart3dRelease = DynamicLookup::instance().getFunc<void(*)(NativeChart3dPtr)>("hebiChartsChart3dRelease");
    hebiChartsChart3dRelease(ptr_);
  }
}
inline Chart3d::Chart3d(Chart3d&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Chart3d& Chart3d::operator=(Chart3d&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Chart3d::~Chart3d() {
  cleanup();
}

// Chart3dModel
inline size_t Chart3dModel::getNumJoints() {
  static auto hebiChartsChart3dModelGetNumJoints = DynamicLookup::instance().getFunc<size_t(*)(NativeChart3dModelPtr)>("hebiChartsChart3dModelGetNumJoints");
  return hebiChartsChart3dModelGetNumJoints(ptr_);
}
inline void Chart3dModel::setPositions(const std::vector<double>& positions) {
   setPositions(positions.data(), positions.size());
}
inline void Chart3dModel::setPositions(const double* positions, size_t length) {
  static auto hebiChartsChart3dModelSetPositions = DynamicLookup::instance().getFunc<void(*)(NativeChart3dModelPtr, const double*, size_t)>("hebiChartsChart3dModelSetPositions");
  hebiChartsChart3dModelSetPositions(ptr_, positions, length);
}
inline void Chart3dModel::setOrientation(double qw, double qx, double qy, double qz) {
  static auto hebiChartsChart3dModelSetOrientation = DynamicLookup::instance().getFunc<void(*)(NativeChart3dModelPtr, double, double, double, double)>("hebiChartsChart3dModelSetOrientation");
  hebiChartsChart3dModelSetOrientation(ptr_, qw, qx, qy, qz);
}
inline void Chart3dModel::setTranslation(double x, double y, double z) {
  static auto hebiChartsChart3dModelSetTranslation = DynamicLookup::instance().getFunc<void(*)(NativeChart3dModelPtr, double, double, double)>("hebiChartsChart3dModelSetTranslation");
  hebiChartsChart3dModelSetTranslation(ptr_, x, y, z);
}
inline void Chart3dModel::setTransform4x4(const std::vector<double>& matrix) {
   setTransform4x4(matrix.data(), matrix.size());
}
inline void Chart3dModel::setTransform4x4(const double* matrix, size_t length) {
  static auto hebiChartsChart3dModelSetTransform4x4 = DynamicLookup::instance().getFunc<void(*)(NativeChart3dModelPtr, const double*, size_t)>("hebiChartsChart3dModelSetTransform4x4");
  hebiChartsChart3dModelSetTransform4x4(ptr_, matrix, length);
}
inline void Chart3dModel::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebiChartsChart3dModelRelease = DynamicLookup::instance().getFunc<void(*)(NativeChart3dModelPtr)>("hebiChartsChart3dModelRelease");
    hebiChartsChart3dModelRelease(ptr_);
  }
}
inline Chart3dModel::Chart3dModel(Chart3dModel&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Chart3dModel& Chart3dModel::operator=(Chart3dModel&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Chart3dModel::~Chart3dModel() {
  cleanup();
}

// Chart3dTriad
inline void Chart3dTriad::setOrientation(double qw, double qx, double qy, double qz) {
  static auto hebiChartsChart3dTriadSetOrientation = DynamicLookup::instance().getFunc<void(*)(NativeChart3dTriadPtr, double, double, double, double)>("hebiChartsChart3dTriadSetOrientation");
  hebiChartsChart3dTriadSetOrientation(ptr_, qw, qx, qy, qz);
}
inline void Chart3dTriad::setTranslation(double x, double y, double z) {
  static auto hebiChartsChart3dTriadSetTranslation = DynamicLookup::instance().getFunc<void(*)(NativeChart3dTriadPtr, double, double, double)>("hebiChartsChart3dTriadSetTranslation");
  hebiChartsChart3dTriadSetTranslation(ptr_, x, y, z);
}
inline void Chart3dTriad::setTransform4x4(const std::vector<double>& matrix) {
   setTransform4x4(matrix.data(), matrix.size());
}
inline void Chart3dTriad::setTransform4x4(const double* matrix, size_t length) {
  static auto hebiChartsChart3dTriadSetTransform4x4 = DynamicLookup::instance().getFunc<void(*)(NativeChart3dTriadPtr, const double*, size_t)>("hebiChartsChart3dTriadSetTransform4x4");
  hebiChartsChart3dTriadSetTransform4x4(ptr_, matrix, length);
}
inline void Chart3dTriad::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebiChartsChart3dTriadRelease = DynamicLookup::instance().getFunc<void(*)(NativeChart3dTriadPtr)>("hebiChartsChart3dTriadRelease");
    hebiChartsChart3dTriadRelease(ptr_);
  }
}
inline Chart3dTriad::Chart3dTriad(Chart3dTriad&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Chart3dTriad& Chart3dTriad::operator=(Chart3dTriad&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Chart3dTriad::~Chart3dTriad() {
  cleanup();
}

// Axis
inline void Axis::setName(const std::string& name) {
   setName(name.c_str());
}
inline void Axis::setName(const char* name) {
  static auto hebiChartsAxisSetName = DynamicLookup::instance().getFunc<void(*)(NativeChartAxisPtr, const char*)>("hebiChartsAxisSetName");
  hebiChartsAxisSetName(ptr_, name);
}
inline void Axis::setUnit(const std::string& unit) {
   setUnit(unit.c_str());
}
inline void Axis::setUnit(const char* unit) {
  static auto hebiChartsAxisSetUnit = DynamicLookup::instance().getFunc<void(*)(NativeChartAxisPtr, const char*)>("hebiChartsAxisSetUnit");
  hebiChartsAxisSetUnit(ptr_, unit);
}
inline void Axis::setLimits(double min, double max) {
  static auto hebiChartsAxisSetLimits = DynamicLookup::instance().getFunc<void(*)(NativeChartAxisPtr, double, double)>("hebiChartsAxisSetLimits");
  hebiChartsAxisSetLimits(ptr_, min, max);
}
inline void Axis::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebiChartsAxisRelease = DynamicLookup::instance().getFunc<void(*)(NativeChartAxisPtr)>("hebiChartsAxisRelease");
    hebiChartsAxisRelease(ptr_);
  }
}
inline Axis::Axis(Axis&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Axis& Axis::operator=(Axis&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Axis::~Axis() {
  cleanup();
}

// Dataset
inline void Dataset::setName(const std::string& name) {
  setName(name.c_str());
}
inline void Dataset::setName(const char* name) {
  static auto hebiChartsDatasetSetName = DynamicLookup::instance().getFunc<void(*)(NativeChartDatasetPtr, const char*)>("hebiChartsDatasetSetName");
  hebiChartsDatasetSetName(ptr_, name);
}
inline void Dataset::setData(const std::vector<double>& x, const std::vector<double>& y) {
  setData(x.data(), y.data(), (std::min)(x.size(), y.size()));
}
inline void Dataset::setData(const double* x, const double* y, size_t length) {
  static auto hebiChartsDatasetSetData = DynamicLookup::instance().getFunc<void(*)(NativeChartDatasetPtr, const double*, const double*, size_t)>("hebiChartsDatasetSetData");
  hebiChartsDatasetSetData(ptr_, x, y, length);
}
inline void Dataset::addPoint(double x, double y) {
  static auto hebiChartsDatasetAddPoint = DynamicLookup::instance().getFunc<void(*)(NativeChartDatasetPtr, double, double)>("hebiChartsDatasetAddPoint");
  hebiChartsDatasetAddPoint(ptr_, x, y);
}
inline void Dataset::setColor(Color color) {
  static auto hebiChartsDatasetSetColor = DynamicLookup::instance().getFunc<void(*)(NativeChartDatasetPtr, Color)>("hebiChartsDatasetSetColor");
  hebiChartsDatasetSetColor(ptr_, color);
}
inline void Dataset::setLineStyle(LineStyle lineStyle) {
  static auto hebiChartsDatasetSetLineStyle = DynamicLookup::instance().getFunc<void(*)(NativeChartDatasetPtr, LineStyle)>("hebiChartsDatasetSetLineStyle");
  hebiChartsDatasetSetLineStyle(ptr_, lineStyle);
}
inline void Dataset::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebiChartsDatasetRelease = DynamicLookup::instance().getFunc<void(*)(NativeChartDatasetPtr)>("hebiChartsDatasetRelease");
    hebiChartsDatasetRelease(ptr_);
  }
}
inline Dataset::Dataset(Dataset&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Dataset& Dataset::operator=(Dataset&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Dataset::~Dataset() {
  cleanup();
}

// Framework
inline bool framework::setTheme(Theme theme) {
  static auto hebiChartsFrameworkSetTheme = DynamicLookup::instance().getFunc<bool(*)(Theme)>("hebiChartsFrameworkSetTheme");
  return hebiChartsFrameworkSetTheme(theme);
}
inline void framework::setAutoCloseWindows(bool autoClose) {
  static auto hebiChartsFrameworkSetAutoCloseWindows = DynamicLookup::instance().getFunc<void(*)(bool)>("hebiChartsFrameworkSetAutoCloseWindows");
  hebiChartsFrameworkSetAutoCloseWindows(autoClose);
}
inline void framework::waitUntilWindowsClosed() {
  static auto hebiChartsFrameworkWaitUntilWindowsClosed = DynamicLookup::instance().getFunc<void(*)()>("hebiChartsFrameworkWaitUntilWindowsClosed");
  hebiChartsFrameworkWaitUntilWindowsClosed();
}

// ==== Cocoa utilities for supporting macOS ====
typedef int (*hebiCharts_main_func_t)(int argc, char** argv);
inline int runApplication(hebiCharts_main_func_t callback, int argc, char** argv) {
  if (!DynamicLookup::instance().isLoaded()) {
    return callback(argc, argv);
  }
  static auto hebiChartsRunApplication = DynamicLookup::instance().getFunc<int(*)(hebiCharts_main_func_t, int, char**)>("hebiChartsRunApplication");
  return hebiChartsRunApplication(callback, argc, argv);
}

// Library Information
namespace lib {

struct Version {
  Version(int major, int minor, int patch, int build) : major_(major), minor_(minor), patch_(patch), build_(build) {}
  int major_{};
  int minor_{};
  int patch_{};
  int build_{};
};

inline Version getHeaderVersion() {
  return {0, 1, 0, 49};
}

inline Version getLibraryVersion() {
  Version v(0,0,0,0);
  static auto hebiChartsGetLibraryVersion = DynamicLookup::instance().getFunc<void(*)(int*, int*, int*, int*)>("hebiChartsGetLibraryVersion");
  hebiChartsGetLibraryVersion(&v.major_, &v.minor_, &v.patch_, &v.build_);
  return v;
}

inline bool isAvailable() { return DynamicLookup::instance().isLoaded(); }

} // namespace lib


template<typename FuncType> FuncType DynamicLookup::getFunc(const char *func) const {
  assert(lib_ != nullptr && "Fatal error -- cannot call hebi::charts functions on unloaded library");
  if (!lib_) {
    std::cerr << "Fatal error -- cannot call hebi::charts functions on unloaded library\n";
    return nullptr;
  }
#ifdef WIN32
  FARPROC symbol = ::GetProcAddress((HMODULE) lib_, func);
#else
  void* symbol = dlsym(lib_, func);
#endif
  if (!symbol) {
    std::cerr << "Fatal error -- hebi::charts library is missing expected symbol: " << func << "\n";
  }
  assert(symbol != nullptr);
  return reinterpret_cast<FuncType>(symbol);
}

DynamicLookup:: DynamicLookup(const std::string &path) {
  path_ = path;
#ifdef WIN32
  // convert to wide string to support non-ascii paths
  int size_needed = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), (int)path.size(), NULL, 0);
  std::wstring wstrTo(size_needed, 0);
  MultiByteToWideChar(CP_UTF8, 0, path.c_str(), (int)path.size(), &wstrTo[0], size_needed);
  lib_ = ::LoadLibraryW(wstrTo.c_str());
#else
  lib_ = dlopen(path.c_str(), RTLD_LAZY);
#endif
  if (!lib_) {
    std::cerr << "hebi::charts library not found! Include \""<< path <<"\" in library search path.\n";
    return;
  }

  // Check version compatibility. Note that we haven't finished the constructor, so we can't
  // just use the API method as it would cause a recursive call stack.
  lib::Version header_version = lib::getHeaderVersion();
  lib::Version lib_version(-1,0,0,0);
  auto hebiChartsGetLibraryVersion = getFunc<void(*)(int*, int*, int*, int*)>("hebiChartsGetLibraryVersion");
  hebiChartsGetLibraryVersion(&lib_version.major_, &lib_version.minor_, &lib_version.patch_, &lib_version.build_);

  bool is_compatible = (lib_version.major_ == header_version.major_ &&
                        lib_version.minor_ >= header_version.minor_ &&
                        (lib_version.minor_ > header_version.minor_ || lib_version.patch_ >= header_version.patch_));
  if (!is_compatible) {
    std::cerr << "Incompatible hebi::charts library version found!\n";
#ifdef WIN32
    if (!::FreeLibrary((HMODULE)lib_))
      std::cerr << "Error closing hebi::charts library!\n";
#else
    if (dlclose(lib_) != 0)
      std::cerr << "Error closing hebi::charts library!\n";
#endif
    lib_ = nullptr;
  }

}

} // namespace hebi
} // namespace charts
