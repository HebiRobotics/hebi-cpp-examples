#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <optional>
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
class Axis;
class Chart3d;
class GridWindow;
class Line;
class Line3d;
class LineChart;
class Mesh3d;
class Robot3d;
class Triad3d;
using UserCallbackFunction = void (*)(void* userData);

namespace internal {
using AxisPtr = struct Axis_*;
using Chart3dPtr = struct Chart3d_*;
using GridWindowPtr = struct GridWindow_*;
using LinePtr = struct Line_*;
using Line3dPtr = struct Line3d_*;
using LineChartPtr = struct LineChart_*;
using Mesh3dPtr = struct Mesh3d_*;
using Robot3dPtr = struct Robot3d_*;
using Triad3dPtr = struct Triad3d_*;
}

// ==== Class Wrappers ====
class Object3d {
public:
  virtual void setOrientation(double qw, double qx, double qy, double qz) = 0;
  virtual void setTranslation(double x, double y, double z) = 0;
  virtual void setTransform4x4(const std::vector<double>& matrix) = 0;
  virtual void setTransform4x4(const double* matrix) = 0;
  virtual void setVisible(bool visible) = 0;
  virtual ~Object3d() = default;
};

class Axis {
  friend class LineChart;
public:
  void setName(const std::string& name);
  void setName(const char* name);
  void setUnit(const std::string& unit);
  void setUnit(const char* unit);
  void setAutoUnitScaling(bool autoUnitScaling);
  void setLimits(double min, double max);
  Axis(Axis&& from) noexcept;
  Axis& operator=(Axis&& from) noexcept;
  ~Axis();
private:
  void cleanup();
  explicit Axis(internal::AxisPtr cPointer) : ptr_(cPointer) {}
  internal::AxisPtr ptr_{};
};

class Chart3d {
  friend class GridWindow;
public:
  std::optional<Robot3d> addHrdf(const std::string& filePath);
  std::optional<Robot3d> addHrdf(const char* filePath);
  std::optional<Mesh3d> addMesh(const std::string& pathOrUrl);
  std::optional<Mesh3d> addMesh(const char* pathOrUrl);
  std::optional<Triad3d> addTriad(double length);
  std::optional<Line3d> addLine();
  Chart3d(Chart3d&& from) noexcept;
  Chart3d& operator=(Chart3d&& from) noexcept;
  ~Chart3d();
private:
  void cleanup();
  explicit Chart3d(internal::Chart3dPtr cPointer) : ptr_(cPointer) {}
  internal::Chart3dPtr ptr_{};
};

class GridWindow {
public:
  void setSize(int width, int height);
  void setLocation(int xOffset, int yOffset);
  void setTitle(const std::string& title);
  void setTitle(const char* title);
  void show();
  void hide();
  bool isShowing();
  void keepOpen(bool keepOpen);
  void waitUntilClosed();
  static std::optional<GridWindow> create();
  static std::optional<GridWindow> create(int rows, int cols);
  std::optional<LineChart> addLineChart();
  std::optional<LineChart> addLineChart(int row, int col);
  std::optional<LineChart> addLineChart(int row, int col, int rowSpan, int colSpan);
  std::optional<Chart3d> add3dChart();
  std::optional<Chart3d> add3dChart(int row, int col);
  std::optional<Chart3d> add3dChart(int row, int col, int rowSpan, int colSpan);
  GridWindow(GridWindow&& from) noexcept;
  GridWindow& operator=(GridWindow&& from) noexcept;
  ~GridWindow();
private:
  void cleanup();
  explicit GridWindow(internal::GridWindowPtr cPointer) : ptr_(cPointer) {}
  internal::GridWindowPtr ptr_{};
};

class Line {
  friend class LineChart;
public:
  void setData(const std::vector<double>& x, const std::vector<double>& y);
  void setData(const double* x, const double* y, size_t length);
  void setMaxPointCount(int count);
  void addPoint(double x, double y);
  void setName(const std::string& name);
  void setName(const char* name);
  void setColor(Color color);
  void setLineStyle(LineStyle lineStyle);
  Line(Line&& from) noexcept;
  Line& operator=(Line&& from) noexcept;
  ~Line();
private:
  void cleanup();
  explicit Line(internal::LinePtr cPointer) : ptr_(cPointer) {}
  internal::LinePtr ptr_{};
};

class Line3d : public Object3d {
  friend class Chart3d;
public:
  void setOrientation(double qw, double qx, double qy, double qz) override;
  void setTranslation(double x, double y, double z) override;
  void setTransform4x4(const std::vector<double>& matrix) override;
  void setTransform4x4(const double* matrix) override;
  void setVisible(bool visible) override;
  void setData(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z);
  void setData(const double* x, const double* y, const double* z, size_t length);
  void setMaxPointCount(int count);
  void addPoint(double x, double y, double z);
  void setColor(Color color);
  Line3d(Line3d&& from) noexcept;
  Line3d& operator=(Line3d&& from) noexcept;
  ~Line3d();
private:
  void cleanup();
  explicit Line3d(internal::Line3dPtr cPointer) : ptr_(cPointer) {}
  internal::Line3dPtr ptr_{};
};

class LineChart {
  friend class GridWindow;
public:
  void setTitle(const std::string& title);
  void setTitle(const char* title);
  std::optional<Axis> getAxisX();
  std::optional<Axis> getAxisY();
  std::optional<Line> addLine(const std::string& name);
  std::optional<Line> addLine(const std::string& name, const std::vector<double>& x, const std::vector<double>& y);
  std::optional<Line> addLine(const char* name, const double* x, const double* y, size_t length);
  LineChart(LineChart&& from) noexcept;
  LineChart& operator=(LineChart&& from) noexcept;
  ~LineChart();
private:
  void cleanup();
  explicit LineChart(internal::LineChartPtr cPointer) : ptr_(cPointer) {}
  internal::LineChartPtr ptr_{};
};

class Mesh3d : public Object3d {
  friend class Chart3d;
public:
  void setOrientation(double qw, double qx, double qy, double qz) override;
  void setTranslation(double x, double y, double z) override;
  void setTransform4x4(const std::vector<double>& matrix) override;
  void setTransform4x4(const double* matrix) override;
  void setVisible(bool visible) override;
  void setScale(double scaleUnitsToMillimeters);
  void setCentered(bool centered);
  Mesh3d(Mesh3d&& from) noexcept;
  Mesh3d& operator=(Mesh3d&& from) noexcept;
  ~Mesh3d();
private:
  void cleanup();
  explicit Mesh3d(internal::Mesh3dPtr cPointer) : ptr_(cPointer) {}
  internal::Mesh3dPtr ptr_{};
};

class Robot3d : public Object3d {
  friend class Chart3d;
public:
  void setOrientation(double qw, double qx, double qy, double qz) override;
  void setTranslation(double x, double y, double z) override;
  void setTransform4x4(const std::vector<double>& matrix) override;
  void setTransform4x4(const double* matrix) override;
  void setVisible(bool visible) override;
  size_t getNumJoints();
  void setPositions(const std::vector<double>& positions);
  void setPositions(const double* positions, size_t length);
  Robot3d(Robot3d&& from) noexcept;
  Robot3d& operator=(Robot3d&& from) noexcept;
  ~Robot3d();
private:
  void cleanup();
  explicit Robot3d(internal::Robot3dPtr cPointer) : ptr_(cPointer) {}
  internal::Robot3dPtr ptr_{};
};

class Triad3d : public Object3d {
  friend class Chart3d;
public:
  void setOrientation(double qw, double qx, double qy, double qz) override;
  void setTranslation(double x, double y, double z) override;
  void setTransform4x4(const std::vector<double>& matrix) override;
  void setTransform4x4(const double* matrix) override;
  void setVisible(bool visible) override;
  Triad3d(Triad3d&& from) noexcept;
  Triad3d& operator=(Triad3d&& from) noexcept;
  ~Triad3d();
private:
  void cleanup();
  explicit Triad3d(internal::Triad3dPtr cPointer) : ptr_(cPointer) {}
  internal::Triad3dPtr ptr_{};
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
// Axis
inline void Axis::setName(const std::string& name) {
  setName(name.c_str());
}
inline void Axis::setName(const char* name) {
  static auto hebi_charts_Axis_setName = DynamicLookup::instance().getFunc<void(*)(internal::AxisPtr, const char*)>("hebi_charts_Axis_setName");
  hebi_charts_Axis_setName(ptr_, name);
}
inline void Axis::setUnit(const std::string& unit) {
  setUnit(unit.c_str());
}
inline void Axis::setUnit(const char* unit) {
  static auto hebi_charts_Axis_setUnit = DynamicLookup::instance().getFunc<void(*)(internal::AxisPtr, const char*)>("hebi_charts_Axis_setUnit");
  hebi_charts_Axis_setUnit(ptr_, unit);
}
inline void Axis::setAutoUnitScaling(bool autoUnitScaling) {
  static auto hebi_charts_Axis_setAutoUnitScaling = DynamicLookup::instance().getFunc<void(*)(internal::AxisPtr, bool)>("hebi_charts_Axis_setAutoUnitScaling");
  hebi_charts_Axis_setAutoUnitScaling(ptr_, autoUnitScaling);
}
inline void Axis::setLimits(double min, double max) {
  static auto hebi_charts_Axis_setLimits = DynamicLookup::instance().getFunc<void(*)(internal::AxisPtr, double, double)>("hebi_charts_Axis_setLimits");
  hebi_charts_Axis_setLimits(ptr_, min, max);
}
inline void Axis::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Axis_release = DynamicLookup::instance().getFunc<void(*)(internal::AxisPtr)>("hebi_charts_Axis_release");
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

// Chart3d
inline std::optional<Robot3d> Chart3d::addHrdf(const std::string& filePath) {
  return addHrdf(filePath.c_str());
}
inline std::optional<Robot3d> Chart3d::addHrdf(const char* filePath) {
  static auto hebi_charts_Chart3d_addHrdf = DynamicLookup::instance().getFunc<internal::Robot3dPtr(*)(internal::Chart3dPtr, const char*)>("hebi_charts_Chart3d_addHrdf");
  auto ptr = hebi_charts_Chart3d_addHrdf(ptr_, filePath);
  return !ptr ? std::nullopt : std::make_optional(Robot3d(ptr));
}
inline std::optional<Mesh3d> Chart3d::addMesh(const std::string& pathOrUrl) {
  return addMesh(pathOrUrl.c_str());
}
inline std::optional<Mesh3d> Chart3d::addMesh(const char* pathOrUrl) {
  static auto hebi_charts_Chart3d_addMesh = DynamicLookup::instance().getFunc<internal::Mesh3dPtr(*)(internal::Chart3dPtr, const char*)>("hebi_charts_Chart3d_addMesh");
  auto ptr = hebi_charts_Chart3d_addMesh(ptr_, pathOrUrl);
  return !ptr ? std::nullopt : std::make_optional(Mesh3d(ptr));
}
inline std::optional<Triad3d> Chart3d::addTriad(double length) {
  static auto hebi_charts_Chart3d_addTriad = DynamicLookup::instance().getFunc<internal::Triad3dPtr(*)(internal::Chart3dPtr, double)>("hebi_charts_Chart3d_addTriad");
  auto ptr = hebi_charts_Chart3d_addTriad(ptr_, length);
  return !ptr ? std::nullopt : std::make_optional(Triad3d(ptr));
}
inline std::optional<Line3d> Chart3d::addLine() {
  static auto hebi_charts_Chart3d_addLine = DynamicLookup::instance().getFunc<internal::Line3dPtr(*)(internal::Chart3dPtr)>("hebi_charts_Chart3d_addLine");
  auto ptr = hebi_charts_Chart3d_addLine(ptr_);
  return !ptr ? std::nullopt : std::make_optional(Line3d(ptr));
}
inline void Chart3d::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Chart3d_release = DynamicLookup::instance().getFunc<void(*)(internal::Chart3dPtr)>("hebi_charts_Chart3d_release");
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

// GridWindow
inline void GridWindow::setSize(int width, int height) {
  static auto hebi_charts_GridWindow_setSize = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr, int, int)>("hebi_charts_GridWindow_setSize");
  hebi_charts_GridWindow_setSize(ptr_, width, height);
}
inline void GridWindow::setLocation(int xOffset, int yOffset) {
  static auto hebi_charts_GridWindow_setLocation = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr, int, int)>("hebi_charts_GridWindow_setLocation");
  hebi_charts_GridWindow_setLocation(ptr_, xOffset, yOffset);
}
inline void GridWindow::setTitle(const std::string& title) {
  setTitle(title.c_str());
}
inline void GridWindow::setTitle(const char* title) {
  static auto hebi_charts_GridWindow_setTitle = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr, const char*)>("hebi_charts_GridWindow_setTitle");
  hebi_charts_GridWindow_setTitle(ptr_, title);
}
inline void GridWindow::show() {
  static auto hebi_charts_GridWindow_show = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr)>("hebi_charts_GridWindow_show");
  hebi_charts_GridWindow_show(ptr_);
}
inline void GridWindow::hide() {
  static auto hebi_charts_GridWindow_hide = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr)>("hebi_charts_GridWindow_hide");
  hebi_charts_GridWindow_hide(ptr_);
}
inline bool GridWindow::isShowing() {
  static auto hebi_charts_GridWindow_isShowing = DynamicLookup::instance().getFunc<bool(*)(internal::GridWindowPtr)>("hebi_charts_GridWindow_isShowing");
  return hebi_charts_GridWindow_isShowing(ptr_);
}
inline void GridWindow::keepOpen(bool keepOpen) {
  static auto hebi_charts_GridWindow_keepOpen = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr, bool)>("hebi_charts_GridWindow_keepOpen");
  hebi_charts_GridWindow_keepOpen(ptr_, keepOpen);
}
inline void GridWindow::waitUntilClosed() {
  static auto hebi_charts_GridWindow_waitUntilClosed = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr)>("hebi_charts_GridWindow_waitUntilClosed");
  hebi_charts_GridWindow_waitUntilClosed(ptr_);
}
inline std::optional<GridWindow> GridWindow::create() {
  return create(1, 1);
}
inline std::optional<GridWindow> GridWindow::create(int rows, int cols) {
  static auto hebi_charts_GridWindow_create = DynamicLookup::instance().getFunc<internal::GridWindowPtr(*)(int, int)>("hebi_charts_GridWindow_create");
  auto ptr = hebi_charts_GridWindow_create(rows, cols);
  return !ptr ? std::nullopt : std::make_optional(GridWindow(ptr));
}
inline std::optional<LineChart> GridWindow::addLineChart() {
  return addLineChart(0, 0);
}
inline std::optional<LineChart> GridWindow::addLineChart(int row, int col) {
  return addLineChart(row, col, 1, 1);
}
inline std::optional<LineChart> GridWindow::addLineChart(int row, int col, int rowSpan, int colSpan) {
  static auto hebi_charts_GridWindow_addLineChart = DynamicLookup::instance().getFunc<internal::LineChartPtr(*)(internal::GridWindowPtr, int, int, int, int)>("hebi_charts_GridWindow_addLineChart");
  auto ptr = hebi_charts_GridWindow_addLineChart(ptr_, row, col, rowSpan, colSpan);
  return !ptr ? std::nullopt : std::make_optional(LineChart(ptr));
}
inline std::optional<Chart3d> GridWindow::add3dChart() {
  return add3dChart(0, 0);
}
inline std::optional<Chart3d> GridWindow::add3dChart(int row, int col) {
  return add3dChart(row, col, 1, 1);
}
inline std::optional<Chart3d> GridWindow::add3dChart(int row, int col, int rowSpan, int colSpan) {
  static auto hebi_charts_GridWindow_add3dChart = DynamicLookup::instance().getFunc<internal::Chart3dPtr(*)(internal::GridWindowPtr, int, int, int, int)>("hebi_charts_GridWindow_add3dChart");
  auto ptr = hebi_charts_GridWindow_add3dChart(ptr_, row, col, rowSpan, colSpan);
  return !ptr ? std::nullopt : std::make_optional(Chart3d(ptr));
}
inline void GridWindow::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_GridWindow_release = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr)>("hebi_charts_GridWindow_release");
    hebi_charts_GridWindow_release(ptr_);
  }
}
inline GridWindow::GridWindow(GridWindow&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline GridWindow& GridWindow::operator=(GridWindow&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline GridWindow::~GridWindow() {
  cleanup();
}

// Line
inline void Line::setData(const std::vector<double>& x, const std::vector<double>& y) {
  setData(x.data(), y.data(), (std::min)(x.size(), y.size()));
}
inline void Line::setData(const double* x, const double* y, size_t length) {
  static auto hebi_charts_Line_setData = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, const double*, const double*, size_t)>("hebi_charts_Line_setData");
  hebi_charts_Line_setData(ptr_, x, y, length);
}
inline void Line::setMaxPointCount(int count) {
  static auto hebi_charts_Line_setMaxPointCount = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, int)>("hebi_charts_Line_setMaxPointCount");
  hebi_charts_Line_setMaxPointCount(ptr_, count);
}
inline void Line::addPoint(double x, double y) {
  static auto hebi_charts_Line_addPoint = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, double, double)>("hebi_charts_Line_addPoint");
  hebi_charts_Line_addPoint(ptr_, x, y);
}
inline void Line::setName(const std::string& name) {
  setName(name.c_str());
}
inline void Line::setName(const char* name) {
  static auto hebi_charts_Line_setName = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, const char*)>("hebi_charts_Line_setName");
  hebi_charts_Line_setName(ptr_, name);
}
inline void Line::setColor(Color color) {
  static auto hebi_charts_Line_setColor = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, Color)>("hebi_charts_Line_setColor");
  hebi_charts_Line_setColor(ptr_, color);
}
inline void Line::setLineStyle(LineStyle lineStyle) {
  static auto hebi_charts_Line_setLineStyle = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, LineStyle)>("hebi_charts_Line_setLineStyle");
  hebi_charts_Line_setLineStyle(ptr_, lineStyle);
}
inline void Line::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Line_release = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr)>("hebi_charts_Line_release");
    hebi_charts_Line_release(ptr_);
  }
}
inline Line::Line(Line&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Line& Line::operator=(Line&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Line::~Line() {
  cleanup();
}

// Line3d
inline void Line3d::setOrientation(double qw, double qx, double qy, double qz) {
  static auto hebi_charts_Line3d_setOrientation = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, double, double, double, double)>("hebi_charts_Line3d_setOrientation");
  hebi_charts_Line3d_setOrientation(ptr_, qw, qx, qy, qz);
}
inline void Line3d::setTranslation(double x, double y, double z) {
  static auto hebi_charts_Line3d_setTranslation = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, double, double, double)>("hebi_charts_Line3d_setTranslation");
  hebi_charts_Line3d_setTranslation(ptr_, x, y, z);
}
inline void Line3d::setTransform4x4(const std::vector<double>& matrix) {
  setTransform4x4(matrix.data());
}
inline void Line3d::setTransform4x4(const double* matrix) {
  static auto hebi_charts_Line3d_setTransform4x4 = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, const double*)>("hebi_charts_Line3d_setTransform4x4");
  hebi_charts_Line3d_setTransform4x4(ptr_, matrix);
}
inline void Line3d::setVisible(bool visible) {
  static auto hebi_charts_Line3d_setVisible = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, bool)>("hebi_charts_Line3d_setVisible");
  hebi_charts_Line3d_setVisible(ptr_, visible);
}
inline void Line3d::setData(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
  setData(x.data(), y.data(), z.data(), (std::min)({x.size(), y.size(), z.size()}));
}
inline void Line3d::setData(const double* x, const double* y, const double* z, size_t length) {
  static auto hebi_charts_Line3d_setData = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, const double*, const double*, const double*, size_t)>("hebi_charts_Line3d_setData");
  hebi_charts_Line3d_setData(ptr_, x, y, z, length);
}
inline void Line3d::setMaxPointCount(int count) {
  static auto hebi_charts_Line3d_setMaxPointCount = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, int)>("hebi_charts_Line3d_setMaxPointCount");
  hebi_charts_Line3d_setMaxPointCount(ptr_, count);
}
inline void Line3d::addPoint(double x, double y, double z) {
  static auto hebi_charts_Line3d_addPoint = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, double, double, double)>("hebi_charts_Line3d_addPoint");
  hebi_charts_Line3d_addPoint(ptr_, x, y, z);
}
inline void Line3d::setColor(Color color) {
  static auto hebi_charts_Line3d_setColor = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, Color)>("hebi_charts_Line3d_setColor");
  hebi_charts_Line3d_setColor(ptr_, color);
}
inline void Line3d::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Line3d_release = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr)>("hebi_charts_Line3d_release");
    hebi_charts_Line3d_release(ptr_);
  }
}
inline Line3d::Line3d(Line3d&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Line3d& Line3d::operator=(Line3d&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Line3d::~Line3d() {
  cleanup();
}

// LineChart
inline void LineChart::setTitle(const std::string& title) {
  setTitle(title.c_str());
}
inline void LineChart::setTitle(const char* title) {
  static auto hebi_charts_LineChart_setTitle = DynamicLookup::instance().getFunc<void(*)(internal::LineChartPtr, const char*)>("hebi_charts_LineChart_setTitle");
  hebi_charts_LineChart_setTitle(ptr_, title);
}
inline std::optional<Axis> LineChart::getAxisX() {
  static auto hebi_charts_LineChart_getAxisX = DynamicLookup::instance().getFunc<internal::AxisPtr(*)(internal::LineChartPtr)>("hebi_charts_LineChart_getAxisX");
  auto ptr = hebi_charts_LineChart_getAxisX(ptr_);
  return !ptr ? std::nullopt : std::make_optional(Axis(ptr));
}
inline std::optional<Axis> LineChart::getAxisY() {
  static auto hebi_charts_LineChart_getAxisY = DynamicLookup::instance().getFunc<internal::AxisPtr(*)(internal::LineChartPtr)>("hebi_charts_LineChart_getAxisY");
  auto ptr = hebi_charts_LineChart_getAxisY(ptr_);
  return !ptr ? std::nullopt : std::make_optional(Axis(ptr));
}
inline std::optional<Line> LineChart::addLine(const std::string& name) {
  return addLine(name.c_str(), nullptr, nullptr, 0);
}
inline std::optional<Line> LineChart::addLine(const std::string& name, const std::vector<double>& x, const std::vector<double>& y) {
  return addLine(name.c_str(), x.data(), y.data(), (std::min)(x.size(), y.size()));
}
inline std::optional<Line> LineChart::addLine(const char* name, const double* x, const double* y, size_t length) {
  static auto hebi_charts_LineChart_addLine = DynamicLookup::instance().getFunc<internal::LinePtr(*)(internal::LineChartPtr, const char*, const double*, const double*, size_t)>("hebi_charts_LineChart_addLine");
  auto ptr = hebi_charts_LineChart_addLine(ptr_, name, x, y, length);
  return !ptr ? std::nullopt : std::make_optional(Line(ptr));
}
inline void LineChart::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_LineChart_release = DynamicLookup::instance().getFunc<void(*)(internal::LineChartPtr)>("hebi_charts_LineChart_release");
    hebi_charts_LineChart_release(ptr_);
  }
}
inline LineChart::LineChart(LineChart&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline LineChart& LineChart::operator=(LineChart&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline LineChart::~LineChart() {
  cleanup();
}

// Mesh3d
inline void Mesh3d::setOrientation(double qw, double qx, double qy, double qz) {
  static auto hebi_charts_Mesh3d_setOrientation = DynamicLookup::instance().getFunc<void(*)(internal::Mesh3dPtr, double, double, double, double)>("hebi_charts_Mesh3d_setOrientation");
  hebi_charts_Mesh3d_setOrientation(ptr_, qw, qx, qy, qz);
}
inline void Mesh3d::setTranslation(double x, double y, double z) {
  static auto hebi_charts_Mesh3d_setTranslation = DynamicLookup::instance().getFunc<void(*)(internal::Mesh3dPtr, double, double, double)>("hebi_charts_Mesh3d_setTranslation");
  hebi_charts_Mesh3d_setTranslation(ptr_, x, y, z);
}
inline void Mesh3d::setTransform4x4(const std::vector<double>& matrix) {
  setTransform4x4(matrix.data());
}
inline void Mesh3d::setTransform4x4(const double* matrix) {
  static auto hebi_charts_Mesh3d_setTransform4x4 = DynamicLookup::instance().getFunc<void(*)(internal::Mesh3dPtr, const double*)>("hebi_charts_Mesh3d_setTransform4x4");
  hebi_charts_Mesh3d_setTransform4x4(ptr_, matrix);
}
inline void Mesh3d::setVisible(bool visible) {
  static auto hebi_charts_Mesh3d_setVisible = DynamicLookup::instance().getFunc<void(*)(internal::Mesh3dPtr, bool)>("hebi_charts_Mesh3d_setVisible");
  hebi_charts_Mesh3d_setVisible(ptr_, visible);
}
inline void Mesh3d::setScale(double scaleUnitsToMillimeters) {
  static auto hebi_charts_Mesh3d_setScale = DynamicLookup::instance().getFunc<void(*)(internal::Mesh3dPtr, double)>("hebi_charts_Mesh3d_setScale");
  hebi_charts_Mesh3d_setScale(ptr_, scaleUnitsToMillimeters);
}
inline void Mesh3d::setCentered(bool centered) {
  static auto hebi_charts_Mesh3d_setCentered = DynamicLookup::instance().getFunc<void(*)(internal::Mesh3dPtr, bool)>("hebi_charts_Mesh3d_setCentered");
  hebi_charts_Mesh3d_setCentered(ptr_, centered);
}
inline void Mesh3d::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Mesh3d_release = DynamicLookup::instance().getFunc<void(*)(internal::Mesh3dPtr)>("hebi_charts_Mesh3d_release");
    hebi_charts_Mesh3d_release(ptr_);
  }
}
inline Mesh3d::Mesh3d(Mesh3d&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Mesh3d& Mesh3d::operator=(Mesh3d&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Mesh3d::~Mesh3d() {
  cleanup();
}

// Robot3d
inline void Robot3d::setOrientation(double qw, double qx, double qy, double qz) {
  static auto hebi_charts_Robot3d_setOrientation = DynamicLookup::instance().getFunc<void(*)(internal::Robot3dPtr, double, double, double, double)>("hebi_charts_Robot3d_setOrientation");
  hebi_charts_Robot3d_setOrientation(ptr_, qw, qx, qy, qz);
}
inline void Robot3d::setTranslation(double x, double y, double z) {
  static auto hebi_charts_Robot3d_setTranslation = DynamicLookup::instance().getFunc<void(*)(internal::Robot3dPtr, double, double, double)>("hebi_charts_Robot3d_setTranslation");
  hebi_charts_Robot3d_setTranslation(ptr_, x, y, z);
}
inline void Robot3d::setTransform4x4(const std::vector<double>& matrix) {
  setTransform4x4(matrix.data());
}
inline void Robot3d::setTransform4x4(const double* matrix) {
  static auto hebi_charts_Robot3d_setTransform4x4 = DynamicLookup::instance().getFunc<void(*)(internal::Robot3dPtr, const double*)>("hebi_charts_Robot3d_setTransform4x4");
  hebi_charts_Robot3d_setTransform4x4(ptr_, matrix);
}
inline void Robot3d::setVisible(bool visible) {
  static auto hebi_charts_Robot3d_setVisible = DynamicLookup::instance().getFunc<void(*)(internal::Robot3dPtr, bool)>("hebi_charts_Robot3d_setVisible");
  hebi_charts_Robot3d_setVisible(ptr_, visible);
}
inline size_t Robot3d::getNumJoints() {
  static auto hebi_charts_Robot3d_getNumJoints = DynamicLookup::instance().getFunc<size_t(*)(internal::Robot3dPtr)>("hebi_charts_Robot3d_getNumJoints");
  return hebi_charts_Robot3d_getNumJoints(ptr_);
}
inline void Robot3d::setPositions(const std::vector<double>& positions) {
  setPositions(positions.data(), positions.size());
}
inline void Robot3d::setPositions(const double* positions, size_t length) {
  static auto hebi_charts_Robot3d_setPositions = DynamicLookup::instance().getFunc<void(*)(internal::Robot3dPtr, const double*, size_t)>("hebi_charts_Robot3d_setPositions");
  hebi_charts_Robot3d_setPositions(ptr_, positions, length);
}
inline void Robot3d::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Robot3d_release = DynamicLookup::instance().getFunc<void(*)(internal::Robot3dPtr)>("hebi_charts_Robot3d_release");
    hebi_charts_Robot3d_release(ptr_);
  }
}
inline Robot3d::Robot3d(Robot3d&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Robot3d& Robot3d::operator=(Robot3d&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Robot3d::~Robot3d() {
  cleanup();
}

// Triad3d
inline void Triad3d::setOrientation(double qw, double qx, double qy, double qz) {
  static auto hebi_charts_Triad3d_setOrientation = DynamicLookup::instance().getFunc<void(*)(internal::Triad3dPtr, double, double, double, double)>("hebi_charts_Triad3d_setOrientation");
  hebi_charts_Triad3d_setOrientation(ptr_, qw, qx, qy, qz);
}
inline void Triad3d::setTranslation(double x, double y, double z) {
  static auto hebi_charts_Triad3d_setTranslation = DynamicLookup::instance().getFunc<void(*)(internal::Triad3dPtr, double, double, double)>("hebi_charts_Triad3d_setTranslation");
  hebi_charts_Triad3d_setTranslation(ptr_, x, y, z);
}
inline void Triad3d::setTransform4x4(const std::vector<double>& matrix) {
  setTransform4x4(matrix.data());
}
inline void Triad3d::setTransform4x4(const double* matrix) {
  static auto hebi_charts_Triad3d_setTransform4x4 = DynamicLookup::instance().getFunc<void(*)(internal::Triad3dPtr, const double*)>("hebi_charts_Triad3d_setTransform4x4");
  hebi_charts_Triad3d_setTransform4x4(ptr_, matrix);
}
inline void Triad3d::setVisible(bool visible) {
  static auto hebi_charts_Triad3d_setVisible = DynamicLookup::instance().getFunc<void(*)(internal::Triad3dPtr, bool)>("hebi_charts_Triad3d_setVisible");
  hebi_charts_Triad3d_setVisible(ptr_, visible);
}
inline void Triad3d::cleanup() {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Triad3d_release = DynamicLookup::instance().getFunc<void(*)(internal::Triad3dPtr)>("hebi_charts_Triad3d_release");
    hebi_charts_Triad3d_release(ptr_);
  }
}
inline Triad3d::Triad3d(Triad3d&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Triad3d& Triad3d::operator=(Triad3d&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Triad3d::~Triad3d() {
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
  return {0, 4, 0, 64};
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
