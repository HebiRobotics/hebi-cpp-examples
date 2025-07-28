#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
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

enum class Color : int32_t {
  Default = 0,
  Primary = 1,
  PrimaryMuted = 2,
  Red = 3,
  Green = 4,
  Blue = 5,
  Yellow = 6,
  Magenta = 7
};

enum class LineStyle : int32_t {
  Default = 0,
  Solid = 1,
  Dashed = 2,
  Points = 3
};

enum class Theme : int32_t {
  PrimerLight = 0,
  PrimerDark = 1,
  NordLight = 2,
  NordDark = 3,
  CupertinoLight = 4,
  CupertinoDark = 5,
  Dracula = 6
};

// ==== Forward Declarations ====
class Chart;
class Chart3d;
class Chart3dModel;
class Chart3dTriad;
class Axis;
class Dataset;
class Window;
using UserCallbackFunction = void (*)(void* userData);

namespace internal {
using NativeChartPtr = struct NativeChart_*;
using NativeChart3dPtr = struct NativeChart3d_*;
using NativeChart3dModelPtr = struct NativeChart3dModel_*;
using NativeChart3dTriadPtr = struct NativeChart3dTriad_*;
using NativeAxisPtr = struct NativeChartAxis_*;
using NativeDatasetPtr = struct NativeChartDataset_*;
using NativeWindowPtr = struct NativeWindow_*;
}

// ==== C++ API Wrappers ====
class Chart {
  friend class Window;
public:
  void setTitle(const std::string& title);
  void setTitle(const char* title);
  Axis getAxisX();
  Axis getAxisY();
  Dataset addLine(const std::string& name);
  Dataset addLine(const std::string& name, const std::vector<double>& x, const std::vector<double>& y);
  Dataset addLine(const char* name, const double* x, const double* y, size_t length);
  Chart(Chart&& from) noexcept;
  Chart& operator=(Chart&& from) noexcept;
  ~Chart();
private:
  void cleanup();
  explicit Chart(internal::NativeChartPtr cPointer) : ptr_(cPointer) {}
  internal::NativeChartPtr ptr_{};
};

class Chart3d {
  friend class Window;
public:
  Chart3dModel addHrdf(const std::string& filePath);
  Chart3dModel addHrdf(const char* filePath);
  Chart3dTriad addTriad(double length);
  Chart3d(Chart3d&& from) noexcept;
  Chart3d& operator=(Chart3d&& from) noexcept;
  ~Chart3d();
private:
  void cleanup();
  explicit Chart3d(internal::NativeChart3dPtr cPointer) : ptr_(cPointer) {}
  internal::NativeChart3dPtr ptr_{};
};

class Chart3dModel {
  friend class Chart3d;
public:
  void setOrientation(double qw, double qx, double qy, double qz);
  void setTranslation(double x, double y, double z);
  void setTransform4x4(const std::vector<double>& matrix);
  void setTransform4x4(const double* matrix, size_t length);
  void setVisible(bool visible);
  size_t getNumJoints();
  void setGhostedMaterial();
  void setPositions(const std::vector<double>& positions);
  void setPositions(const double* positions, size_t length);
  Chart3dModel(Chart3dModel&& from) noexcept;
  Chart3dModel& operator=(Chart3dModel&& from) noexcept;
  ~Chart3dModel();
private:
  void cleanup();
  explicit Chart3dModel(internal::NativeChart3dModelPtr cPointer) : ptr_(cPointer) {}
  internal::NativeChart3dModelPtr ptr_{};
};

class Chart3dTriad {
  friend class Chart3d;
public:
  void setOrientation(double qw, double qx, double qy, double qz);
  void setTranslation(double x, double y, double z);
  void setTransform4x4(const std::vector<double>& matrix);
  void setTransform4x4(const double* matrix, size_t length);
  void setVisible(bool visible);
  Chart3dTriad(Chart3dTriad&& from) noexcept;
  Chart3dTriad& operator=(Chart3dTriad&& from) noexcept;
  ~Chart3dTriad();
private:
  void cleanup();
  explicit Chart3dTriad(internal::NativeChart3dTriadPtr cPointer) : ptr_(cPointer) {}
  internal::NativeChart3dTriadPtr ptr_{};
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
  explicit Axis(internal::NativeAxisPtr cPointer) : ptr_(cPointer) {}
  internal::NativeAxisPtr ptr_{};
};

class Dataset {
  friend class Chart;
public:
  void setName(const std::string& name);
  void setName(const char* name);
  void setData(const std::vector<double>& x, const std::vector<double>& y);
  void setData(const double* x, const double* y, size_t length);
  void setMaxPointCount(int count);
  void addPoint(double x, double y);
  void setColor(Color color);
  void setLineStyle(LineStyle lineStyle);
  Dataset(Dataset&& from) noexcept;
  Dataset& operator=(Dataset&& from) noexcept;
  ~Dataset();
private:
  void cleanup();
  explicit Dataset(internal::NativeDatasetPtr cPointer) : ptr_(cPointer) {}
  internal::NativeDatasetPtr ptr_{};
};

// Window
class Window {
public:
  Window();
  Chart addChart();
  Chart addChart(int col, int row);
  Chart addChart(int col, int row, int colSpan, int rowSpan);
  Chart3d addChart3d();
  Chart3d addChart3d(int col, int row);
  Chart3d addChart3d(int col, int row, int colSpan, int rowSpan);
  void setSize(int width, int height);
  void setLocation(int xOffset, int yOffset);
  void setGridSize(int numCols, int numRows);
  void setTitle(const std::string& title);
  void setTitle(const char* title);
  void show();
  void hide();
  bool isShowing();
  void keepOpen(bool keepOpen);
  void waitUntilClosed();
  Window(Window&& from) noexcept;
  Window& operator=(Window&& from) noexcept;
  ~Window();
private:
  void cleanup();
  explicit Window(internal::NativeWindowPtr cPointer) : ptr_(cPointer) {}
  internal::NativeWindowPtr ptr_{};
};

namespace framework {
bool setTheme(Theme theme);
void setAutoCloseWindows(bool autoClose);
void waitUntilWindowsClosed();
void runOnUiThread(UserCallbackFunction func, void* userData);
void printLastErrorDetails();
void printThreadInfo(const std::string& name);
void printThreadInfo(const char* name);
} // namespace framework

// ==== C Library Lookup ====
namespace {
struct DynamicLookup {
public:
  static DynamicLookup& instance() {
#ifdef WIN32
    static DynamicLookup lib("hebi_charts.dll");
#elif __APPLE__
    static DynamicLookup lib("libhebi_charts.dylib");
#else
    static DynamicLookup lib("./libhebi_charts.so", "libhebi_charts.so");
#endif
    return lib;
  }

  bool isLoaded() const { return lib_ != nullptr; };

  template<typename FuncType> FuncType getFunc(const char* func) const;
  DynamicLookup(const DynamicLookup&) = delete;
  DynamicLookup &operator = (const DynamicLookup&) = delete;

private:
  explicit DynamicLookup(const std::string& path, const std::string& alt_path = "");
  void* lib_{};
  std::string path_{};
};
} // namespace

// ==== C++ Implementations ====
// Chart
inline void Chart::setTitle(const std::string& title) {
  setTitle(title.c_str());
}
inline void Chart::setTitle(const char* title) {
  static auto hebi_charts_Chart_setTitle = DynamicLookup::instance().getFunc<void(*)(internal::NativeChartPtr, const char*)>("hebi_charts_Chart_setTitle");
  hebi_charts_Chart_setTitle(ptr_, title);
}
inline Axis Chart::getAxisX() {
  static auto hebi_charts_Chart_getAxisX = DynamicLookup::instance().getFunc<internal::NativeAxisPtr(*)(internal::NativeChartPtr)>("hebi_charts_Chart_getAxisX");
  return Axis(hebi_charts_Chart_getAxisX(ptr_));
}
inline Axis Chart::getAxisY() {
  static auto hebi_charts_Chart_getAxisY = DynamicLookup::instance().getFunc<internal::NativeAxisPtr(*)(internal::NativeChartPtr)>("hebi_charts_Chart_getAxisY");
  return Axis(hebi_charts_Chart_getAxisY(ptr_));
}
inline Dataset Chart::addLine(const std::string& name) {
  return addLine(name.c_str(), nullptr, nullptr, 0);
}
inline Dataset Chart::addLine(const std::string& name, const std::vector<double>& x, const std::vector<double>& y) {
  return addLine(name.c_str(), x.data(), y.data(), (std::min)(x.size(), y.size()));
}
inline Dataset Chart::addLine(const char* name, const double* x, const double* y, size_t length) {
  static auto hebi_charts_Chart_addLine = DynamicLookup::instance().getFunc<internal::NativeDatasetPtr(*)(internal::NativeChartPtr, const char*, const double*, const double*, size_t)>("hebi_charts_Chart_addLine");
  return Dataset(hebi_charts_Chart_addLine(ptr_, name, x, y, length));
}
inline void Chart::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Chart_release = DynamicLookup::instance().getFunc<void(*)(internal::NativeChartPtr)>("hebi_charts_Chart_release");
    hebi_charts_Chart_release(ptr_);
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
inline Chart3dModel Chart3d::addHrdf(const std::string& filePath) {
  return addHrdf(filePath.c_str());
}
inline Chart3dModel Chart3d::addHrdf(const char* filePath) {
  static auto hebi_charts_Chart3d_addHrdf = DynamicLookup::instance().getFunc<internal::NativeChart3dModelPtr(*)(internal::NativeChart3dPtr, const char*)>("hebi_charts_Chart3d_addHrdf");
  return Chart3dModel(hebi_charts_Chart3d_addHrdf(ptr_, filePath));
}
inline Chart3dTriad Chart3d::addTriad(double length) {
  static auto hebi_charts_Chart3d_addTriad = DynamicLookup::instance().getFunc<internal::NativeChart3dTriadPtr(*)(internal::NativeChart3dPtr, double)>("hebi_charts_Chart3d_addTriad");
  return Chart3dTriad(hebi_charts_Chart3d_addTriad(ptr_, length));
}
inline void Chart3d::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Chart3d_release = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dPtr)>("hebi_charts_Chart3d_release");
    hebi_charts_Chart3d_release(ptr_);
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
inline void Chart3dModel::setOrientation(double qw, double qx, double qy, double qz) {
  static auto hebi_charts_Chart3dModel_setOrientation = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dModelPtr, double, double, double, double)>("hebi_charts_Chart3dModel_setOrientation");
  hebi_charts_Chart3dModel_setOrientation(ptr_, qw, qx, qy, qz);
}
inline void Chart3dModel::setTranslation(double x, double y, double z) {
  static auto hebi_charts_Chart3dModel_setTranslation = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dModelPtr, double, double, double)>("hebi_charts_Chart3dModel_setTranslation");
  hebi_charts_Chart3dModel_setTranslation(ptr_, x, y, z);
}
inline void Chart3dModel::setTransform4x4(const std::vector<double>& matrix) {
  setTransform4x4(matrix.data(), matrix.size());
}
inline void Chart3dModel::setTransform4x4(const double* matrix, size_t length) {
  static auto hebi_charts_Chart3dModel_setTransform4x4 = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dModelPtr, const double*, size_t)>("hebi_charts_Chart3dModel_setTransform4x4");
  hebi_charts_Chart3dModel_setTransform4x4(ptr_, matrix, length);
}
inline void Chart3dModel::setVisible(bool visible) {
  static auto hebi_charts_Chart3dModel_setVisible = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dModelPtr, bool)>("hebi_charts_Chart3dModel_setVisible");
  hebi_charts_Chart3dModel_setVisible(ptr_, visible);
}
inline size_t Chart3dModel::getNumJoints() {
  static auto hebi_charts_Chart3dModel_getNumJoints = DynamicLookup::instance().getFunc<size_t(*)(internal::NativeChart3dModelPtr)>("hebi_charts_Chart3dModel_getNumJoints");
  return hebi_charts_Chart3dModel_getNumJoints(ptr_);
}
inline void Chart3dModel::setGhostedMaterial() {
  static auto hebi_charts_Chart3dModel_setGhostedMaterial = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dModelPtr)>("hebi_charts_Chart3dModel_setGhostedMaterial");
  hebi_charts_Chart3dModel_setGhostedMaterial(ptr_);
}
inline void Chart3dModel::setPositions(const std::vector<double>& positions) {
  setPositions(positions.data(), positions.size());
}
inline void Chart3dModel::setPositions(const double* positions, size_t length) {
  static auto hebi_charts_Chart3dModel_setPositions = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dModelPtr, const double*, size_t)>("hebi_charts_Chart3dModel_setPositions");
  hebi_charts_Chart3dModel_setPositions(ptr_, positions, length);
}
inline void Chart3dModel::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Chart3dModel_release = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dModelPtr)>("hebi_charts_Chart3dModel_release");
    hebi_charts_Chart3dModel_release(ptr_);
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
  static auto hebi_charts_Chart3dTriad_setOrientation = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dTriadPtr, double, double, double, double)>("hebi_charts_Chart3dTriad_setOrientation");
  hebi_charts_Chart3dTriad_setOrientation(ptr_, qw, qx, qy, qz);
}
inline void Chart3dTriad::setTranslation(double x, double y, double z) {
  static auto hebi_charts_Chart3dTriad_setTranslation = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dTriadPtr, double, double, double)>("hebi_charts_Chart3dTriad_setTranslation");
  hebi_charts_Chart3dTriad_setTranslation(ptr_, x, y, z);
}
inline void Chart3dTriad::setTransform4x4(const std::vector<double>& matrix) {
  setTransform4x4(matrix.data(), matrix.size());
}
inline void Chart3dTriad::setTransform4x4(const double* matrix, size_t length) {
  static auto hebi_charts_Chart3dTriad_setTransform4x4 = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dTriadPtr, const double*, size_t)>("hebi_charts_Chart3dTriad_setTransform4x4");
  hebi_charts_Chart3dTriad_setTransform4x4(ptr_, matrix, length);
}
inline void Chart3dTriad::setVisible(bool visible) {
  static auto hebi_charts_Chart3dTriad_setVisible = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dTriadPtr, bool)>("hebi_charts_Chart3dTriad_setVisible");
  hebi_charts_Chart3dTriad_setVisible(ptr_, visible);
}
inline void Chart3dTriad::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Chart3dTriad_release = DynamicLookup::instance().getFunc<void(*)(internal::NativeChart3dTriadPtr)>("hebi_charts_Chart3dTriad_release");
    hebi_charts_Chart3dTriad_release(ptr_);
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
  static auto hebi_charts_Axis_setName = DynamicLookup::instance().getFunc<void(*)(internal::NativeAxisPtr, const char*)>("hebi_charts_Axis_setName");
  hebi_charts_Axis_setName(ptr_, name);
}
inline void Axis::setUnit(const std::string& unit) {
  setUnit(unit.c_str());
}
inline void Axis::setUnit(const char* unit) {
  static auto hebi_charts_Axis_setUnit = DynamicLookup::instance().getFunc<void(*)(internal::NativeAxisPtr, const char*)>("hebi_charts_Axis_setUnit");
  hebi_charts_Axis_setUnit(ptr_, unit);
}
inline void Axis::setLimits(double min, double max) {
  static auto hebi_charts_Axis_setLimits = DynamicLookup::instance().getFunc<void(*)(internal::NativeAxisPtr, double, double)>("hebi_charts_Axis_setLimits");
  hebi_charts_Axis_setLimits(ptr_, min, max);
}
inline void Axis::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Axis_release = DynamicLookup::instance().getFunc<void(*)(internal::NativeAxisPtr)>("hebi_charts_Axis_release");
    hebi_charts_Axis_release(ptr_);
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
  static auto hebi_charts_Dataset_setName = DynamicLookup::instance().getFunc<void(*)(internal::NativeDatasetPtr, const char*)>("hebi_charts_Dataset_setName");
  hebi_charts_Dataset_setName(ptr_, name);
}
inline void Dataset::setData(const std::vector<double>& x, const std::vector<double>& y) {
  setData(x.data(), y.data(), (std::min)(x.size(), y.size()));
}
inline void Dataset::setData(const double* x, const double* y, size_t length) {
  static auto hebi_charts_Dataset_setData = DynamicLookup::instance().getFunc<void(*)(internal::NativeDatasetPtr, const double*, const double*, size_t)>("hebi_charts_Dataset_setData");
  hebi_charts_Dataset_setData(ptr_, x, y, length);
}
inline void Dataset::setMaxPointCount(int count) {
  static auto hebi_charts_Dataset_setMaxPointCount = DynamicLookup::instance().getFunc<void(*)(internal::NativeDatasetPtr, int)>("hebi_charts_Dataset_setMaxPointCount");
  hebi_charts_Dataset_setMaxPointCount(ptr_, count);
}
inline void Dataset::addPoint(double x, double y) {
  static auto hebi_charts_Dataset_addPoint = DynamicLookup::instance().getFunc<void(*)(internal::NativeDatasetPtr, double, double)>("hebi_charts_Dataset_addPoint");
  hebi_charts_Dataset_addPoint(ptr_, x, y);
}
inline void Dataset::setColor(Color color) {
  static auto hebi_charts_Dataset_setColor = DynamicLookup::instance().getFunc<void(*)(internal::NativeDatasetPtr, Color)>("hebi_charts_Dataset_setColor");
  hebi_charts_Dataset_setColor(ptr_, color);
}
inline void Dataset::setLineStyle(LineStyle lineStyle) {
  static auto hebi_charts_Dataset_setLineStyle = DynamicLookup::instance().getFunc<void(*)(internal::NativeDatasetPtr, LineStyle)>("hebi_charts_Dataset_setLineStyle");
  hebi_charts_Dataset_setLineStyle(ptr_, lineStyle);
}
inline void Dataset::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Dataset_release = DynamicLookup::instance().getFunc<void(*)(internal::NativeDatasetPtr)>("hebi_charts_Dataset_release");
    hebi_charts_Dataset_release(ptr_);
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

// Window
inline Window::Window() {
  static auto hebi_charts_Window_create = DynamicLookup::instance().getFunc<internal::NativeWindowPtr(*)()>("hebi_charts_Window_create");
  ptr_ = hebi_charts_Window_create();
}
inline Chart Window::addChart() {
  return addChart(0, 0);
}
inline Chart Window::addChart(int col, int row) {
  return addChart(col, row, 1, 1);
}
inline Chart Window::addChart(int col, int row, int colSpan, int rowSpan) {
  static auto hebi_charts_Window_addChart = DynamicLookup::instance().getFunc<internal::NativeChartPtr(*)(internal::NativeWindowPtr, int, int, int, int)>("hebi_charts_Window_addChart");
  return Chart(hebi_charts_Window_addChart(ptr_, col, row, colSpan, rowSpan));
}
inline Chart3d Window::addChart3d() {
  return addChart3d(0, 0);
}
inline Chart3d Window::addChart3d(int col, int row) {
  return addChart3d(col, row, 1, 1);
}
inline Chart3d Window::addChart3d(int col, int row, int colSpan, int rowSpan) {
  static auto hebi_charts_Window_addChart3d = DynamicLookup::instance().getFunc<internal::NativeChart3dPtr(*)(internal::NativeWindowPtr, int, int, int, int)>("hebi_charts_Window_addChart3d");
  return Chart3d(hebi_charts_Window_addChart3d(ptr_, col, row, colSpan, rowSpan));
}
inline void Window::setSize(int width, int height) {
  static auto hebi_charts_Window_setSize = DynamicLookup::instance().getFunc<void(*)(internal::NativeWindowPtr, int, int)>("hebi_charts_Window_setSize");
  hebi_charts_Window_setSize(ptr_, width, height);
}
inline void Window::setLocation(int xOffset, int yOffset) {
  static auto hebi_charts_Window_setLocation = DynamicLookup::instance().getFunc<void(*)(internal::NativeWindowPtr, int, int)>("hebi_charts_Window_setLocation");
  hebi_charts_Window_setLocation(ptr_, xOffset, yOffset);
}
inline void Window::setGridSize(int numCols, int numRows) {
  static auto hebi_charts_Window_setGridSize = DynamicLookup::instance().getFunc<void(*)(internal::NativeWindowPtr, int, int)>("hebi_charts_Window_setGridSize");
  hebi_charts_Window_setGridSize(ptr_, numCols, numRows);
}
inline void Window::setTitle(const std::string& title) {
  setTitle(title.c_str());
}
inline void Window::setTitle(const char* title) {
  static auto hebi_charts_Window_setTitle = DynamicLookup::instance().getFunc<void(*)(internal::NativeWindowPtr, const char*)>("hebi_charts_Window_setTitle");
  hebi_charts_Window_setTitle(ptr_, title);
}
inline void Window::show() {
  static auto hebi_charts_Window_show = DynamicLookup::instance().getFunc<void(*)(internal::NativeWindowPtr)>("hebi_charts_Window_show");
  hebi_charts_Window_show(ptr_);
}
inline void Window::hide() {
  static auto hebi_charts_Window_hide = DynamicLookup::instance().getFunc<void(*)(internal::NativeWindowPtr)>("hebi_charts_Window_hide");
  hebi_charts_Window_hide(ptr_);
}
inline bool Window::isShowing() {
  static auto hebi_charts_Window_isShowing = DynamicLookup::instance().getFunc<bool(*)(internal::NativeWindowPtr)>("hebi_charts_Window_isShowing");
  return hebi_charts_Window_isShowing(ptr_);
}
inline void Window::keepOpen(bool keepOpen) {
  static auto hebi_charts_Window_keepOpen = DynamicLookup::instance().getFunc<void(*)(internal::NativeWindowPtr, bool)>("hebi_charts_Window_keepOpen");
  hebi_charts_Window_keepOpen(ptr_, keepOpen);
}
inline void Window::waitUntilClosed() {
  static auto hebi_charts_Window_waitUntilClosed = DynamicLookup::instance().getFunc<void(*)(internal::NativeWindowPtr)>("hebi_charts_Window_waitUntilClosed");
  hebi_charts_Window_waitUntilClosed(ptr_);
}
inline void Window::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Window_release = DynamicLookup::instance().getFunc<void(*)(internal::NativeWindowPtr)>("hebi_charts_Window_release");
    hebi_charts_Window_release(ptr_);
  }
}
inline Window::Window(Window&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Window& Window::operator=(Window&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Window::~Window() {
  cleanup();
}

// Framework
inline bool framework::setTheme(Theme theme) {
  static auto hebi_charts_Framework_setTheme = DynamicLookup::instance().getFunc<bool(*)(Theme)>("hebi_charts_Framework_setTheme");
  return hebi_charts_Framework_setTheme(theme);
}
inline void framework::setAutoCloseWindows(bool autoClose) {
  static auto hebi_charts_Framework_setAutoCloseWindows = DynamicLookup::instance().getFunc<void(*)(bool)>("hebi_charts_Framework_setAutoCloseWindows");
  hebi_charts_Framework_setAutoCloseWindows(autoClose);
}
inline void framework::waitUntilWindowsClosed() {
  static auto hebi_charts_Framework_waitUntilWindowsClosed = DynamicLookup::instance().getFunc<void(*)()>("hebi_charts_Framework_waitUntilWindowsClosed");
  hebi_charts_Framework_waitUntilWindowsClosed();
}
inline void framework::runOnUiThread(UserCallbackFunction func, void* userData) {
  static auto hebi_charts_Framework_runOnUiThread = DynamicLookup::instance().getFunc<void(*)(UserCallbackFunction, void*)>("hebi_charts_Framework_runOnUiThread");
  hebi_charts_Framework_runOnUiThread(func, userData);
}
inline void framework::printLastErrorDetails() {
  static auto hebi_charts_Framework_printLastErrorDetails = DynamicLookup::instance().getFunc<void(*)()>("hebi_charts_Framework_printLastErrorDetails");
  hebi_charts_Framework_printLastErrorDetails();
}
inline void framework::printThreadInfo(const std::string& name) {
  printThreadInfo(name.c_str());
}
inline void framework::printThreadInfo(const char* name) {
  static auto hebi_charts_Framework_printThreadInfo = DynamicLookup::instance().getFunc<void(*)(const char*)>("hebi_charts_Framework_printThreadInfo");
  hebi_charts_Framework_printThreadInfo(name);
}

// ==== Cocoa utilities for supporting macOS ====
typedef int (*hebi_charts_MainCallbackFunction)(int argc, char** argv);
inline int runApplication(hebi_charts_MainCallbackFunction callback, int argc, char** argv) {
  if (!DynamicLookup::instance().isLoaded()) {
    return callback(argc, argv);
  }
  static auto hebi_charts_runApplication = DynamicLookup::instance().getFunc<int(*)(hebi_charts_MainCallbackFunction, int, char**)>("hebi_charts_runApplication");
  return hebi_charts_runApplication(callback, argc, argv);
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
  return {0, 3, 0, 61};
}

inline Version getLibraryVersion() {
  Version v(0,0,0,0);
  static auto hebi_charts_getLibraryVersion = DynamicLookup::instance().getFunc<void(*)(int*, int*, int*, int*)>("hebi_charts_getLibraryVersion");
  hebi_charts_getLibraryVersion(&v.major_, &v.minor_, &v.patch_, &v.build_);
  return v;
}

inline bool isAvailable() { return DynamicLookup::instance().isLoaded(); }

} // namespace lib


template<typename FuncType> FuncType DynamicLookup::getFunc(const char* func) const {
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

DynamicLookup:: DynamicLookup(const std::string& path, const std::string& alt_path) {
  path_ = path;
#ifdef WIN32
  // convert to wide string to support non-ascii paths
  int size_needed = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), (int)path.size(), NULL, 0);
  std::wstring wstrTo(size_needed, 0);
  MultiByteToWideChar(CP_UTF8, 0, path.c_str(), (int)path.size(), &wstrTo[0], size_needed);
  lib_ = ::LoadLibraryW(wstrTo.c_str());
  if (!lib_ && !alt_path.empty()) {
    size_needed = MultiByteToWideChar(CP_UTF8, 0, alt_path.c_str(), (int)alt_path.size(), NULL, 0);
    wstrTo = std::wstring(size_needed, 0);
    MultiByteToWideChar(CP_UTF8, 0, alt_path.c_str(), (int)alt_path.size(), &wstrTo[0], size_needed);
    lib_ = ::LoadLibraryW(wstrTo.c_str());
  }
#else
  lib_ = dlopen(path.c_str(), RTLD_LAZY);
  if (!lib_ && !alt_path.empty()) {
    lib_ = dlopen(alt_path.c_str(), RTLD_LAZY);
  }
#endif
  if (!lib_) {
    std::cerr << "hebi::charts library not found! Include \""<< path <<"\" in library search path.\n";
    return;
  }

  // Check version compatibility. Note that we haven't finished the constructor, so we can't
  // just use the API method as it would cause a recursive call stack.
  lib::Version header_version = lib::getHeaderVersion();
  lib::Version lib_version(-1,0,0,0);
  auto hebi_charts_getLibraryVersion = getFunc<void(*)(int*, int*, int*, int*)>("hebi_charts_getLibraryVersion");
  hebi_charts_getLibraryVersion(&lib_version.major_, &lib_version.minor_, &lib_version.patch_, &lib_version.build_);

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
