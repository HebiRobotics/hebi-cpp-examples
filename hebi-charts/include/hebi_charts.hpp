#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
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

class Exception : public std::exception {
public:
  Exception(const char* message) noexcept : msg_(message) {}
  Exception(const std::string& message) noexcept : msg_(message) {}
  virtual ~Exception() noexcept {}
  const char* what() const noexcept override {
    return msg_.c_str();
  }
protected:
  std::string msg_;
};

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
class LineChart;
class Object3d;
class Line3d;
class Mesh3d;
class Robot3d;
class Triad3d;
using UserCallbackFunction = void (*)(void* userData);

namespace internal {
using AxisPtr = struct Axis_*;
using Chart3dPtr = struct Chart3d_*;
using GridWindowPtr = struct GridWindow_*;
using LinePtr = struct Line_*;
using LineChartPtr = struct LineChart_*;
using Object3dPtr = struct Object3d_*;
using Line3dPtr = struct Line3d_*;
using Mesh3dPtr = struct Mesh3d_*;
using Robot3dPtr = struct Robot3d_*;
using Triad3dPtr = struct Triad3d_*;
}

// ==== Class Wrappers ====

/**
 * @brief Represents an Axis on an XY chart
 */
class Axis {
  friend class LineChart;
public:

  /**
   * @brief Sets the displayed axis name
   *
   * @param name 
   */
  void setName(const char* name) noexcept;
  void setName(const std::string& name) noexcept;

  /**
   * @brief Sets the displayed axis unit
   *
   * @param unit 
   */
  void setUnit(const char* unit) noexcept;
  void setUnit(const std::string& unit) noexcept;

  /**
   * @brief Enables auto-scaling for Si units, e.g., 0.01s gets displayed as 10ms
   *
   * @param autoUnitScaling true enables auto-scaling
   */
  void setAutoUnitScaling(bool autoUnitScaling) noexcept;

  /**
   * @brief Sets the min and max limits for this axis. Disables auto-ranging.
   *
   * @param min 
   * @param max 
   */
  void setLimits(double min, double max) noexcept;
  Axis(Axis&& from) noexcept;
  Axis& operator=(Axis&& from) noexcept;
  ~Axis() noexcept;
private:
  void cleanup() noexcept;
  explicit Axis(internal::AxisPtr cPointer) noexcept : ptr_(cPointer) {}
  internal::AxisPtr ptr_{};
};


/**
 * @brief Represents a 3d chart that can render a variety of 3d objects
 */
class Chart3d {
  friend class GridWindow;
public:

  /**
   * @brief Loads a robot from an HRDF (Hebi Robot Description Format) file
   *
   * @param hrdf relative or absolute file path
   * @throw on internal errors
   */
  Robot3d addHrdf(const char* filePath);
  Robot3d addHrdf(const std::string& filePath);

  /**
   * @brief Loads a 3d mesh from a file
   *
   * @param pathOrUrl file path or web-url to an .obj file
   * @throw on internal errors
   */
  Mesh3d addMesh(const char* pathOrUrl);
  Mesh3d addMesh(const std::string& pathOrUrl);

  /**
   * @brief Adds a triad that represents a right-handed coordinate frame
   *
   * @param lengthInMeters length of each axis in [m]
   * @throw on internal errors
   */
  Triad3d addTriad(double lengthInMeters = 0.03);
  Line3d addLine();
  Chart3d(Chart3d&& from) noexcept;
  Chart3d& operator=(Chart3d&& from) noexcept;
  ~Chart3d() noexcept;
private:
  void cleanup() noexcept;
  explicit Chart3d(internal::Chart3dPtr cPointer) noexcept : ptr_(cPointer) {}
  internal::Chart3dPtr ptr_{};
};


/**
 * @brief Represents a window containing an equally sized row/col grid
 */
class GridWindow {
public:

  /**
   * @brief Creates a grid of equally sized rows and columns
   *
   * @param rows 
   * @param cols 
   * @throw on internal errors
   */
  GridWindow(int rows = 1, int cols = 1);

  /**
   * @brief Creates a 2d line chart with the given size
   *
   * @param row 
   * @param col 
   * @param rowSpan 
   * @param colSpan 
   * @throw on internal errors
   */
  LineChart addLineChart(int row = 0, int col = 0, int rowSpan = 1, int colSpan = 1);

  /**
   * @brief Creates a 3d chart with the given size
   *
   * @param row 
   * @param col 
   * @param rowSpan 
   * @param colSpan 
   * @throw on internal errors
   */
  Chart3d addChart3d(int row = 0, int col = 0, int rowSpan = 1, int colSpan = 1);

  /**
   * @brief Sets the window size in pixels
   *
   * @param width 
   * @param height 
   */
  void setSize(int width, int height) noexcept;

  /**
   * @brief Sets the x and y location in pixels
   *
   * @param xOffset 
   * @param yOffset 
   */
  void setLocation(int xOffset, int yOffset) noexcept;

  /**
   * @brief Sets the title of the window header bar
   *
   * @param title 
   */
  void setTitle(const char* title) noexcept;
  void setTitle(const std::string& title) noexcept;

  /**
   * @brief Shows the window. May be called multiple times
   */
  void show() noexcept;

  /**
   * @brief Hides the window. May be called multiple times. Hidden windows are not destroyed
   */
  void hide() noexcept;

  /**
   * @brief Returns whether the window is currently showing
   */
  bool isShowing() const noexcept;

  /**
   * @brief Keeps the window open after the destructor gets called
   *
   * @param keepOpen 
   */
  void keepOpen(bool keepOpen) noexcept;

  /**
   * @brief Waits until this window gets closed by the user
   */
  void waitUntilClosed() const noexcept;
  GridWindow(GridWindow&& from) noexcept;
  GridWindow& operator=(GridWindow&& from) noexcept;
  ~GridWindow() noexcept;
private:
  void cleanup() noexcept;
  internal::GridWindowPtr ptr_{};
};


/**
 * @brief Represents a 2d dataset inside a chart
 */
class Line {
  friend class LineChart;
public:

  /**
   * @brief Sets the rendering color
   *
   * @param color 
   */
  void setColor(Color color) noexcept;

  /**
   * @brief Sets the rendering style
   *
   * @param lineStyle 
   */
  void setLineStyle(LineStyle lineStyle) noexcept;

  /**
   * @details 
   * Sets the internal data to the given values, and sets the
   * maximum point count to the provided length.
   *
   * @param x points
   * @param y points
   * @param length number of x/y points
   */
  void setData(const double* x, const double* y, size_t length) noexcept;
  void setData(const std::vector<double>& x, const std::vector<double>& y) noexcept;

  /**
   * @details 
   * Sets the internal maximum point count for incrementally
   * adding points. May clear existing data.
   *
   * @param count maximum number of points
   */
  void setMaxPointCount(int count) noexcept;

  /**
   * @details 
   * Adds one point to an internal rolling buffer. Once the maximum
   * point count is reached, it will overwrite the earliest data.
   *
   * @param x 
   * @param y 
   */
  void addPoint(double x, double y) noexcept;

  /**
   * @brief Sets the name shown in the chart legend
   *
   * @param name 
   */
  void setName(const char* name) noexcept;
  void setName(const std::string& name) noexcept;
  Line(Line&& from) noexcept;
  Line& operator=(Line&& from) noexcept;
  ~Line() noexcept;
private:
  void cleanup() noexcept;
  explicit Line(internal::LinePtr cPointer) noexcept : ptr_(cPointer) {}
  internal::LinePtr ptr_{};
};


/**
 * @brief Represents an XY chart
 */
class LineChart {
  friend class GridWindow;
public:

  /**
   * @brief Creates a new dataset that represents a line in 2d space
   *
   * @param name 
   * @param x 
   * @param y 
   * @param length 
   * @throw on internal errors
   */
  Line addLine(const char* name, const double* x, const double* y, size_t length);
  Line addLine(const std::string& name);
  Line addLine(const std::string& name, const std::vector<double>& x, const std::vector<double>& y);

  /**
   * @brief Sets the title shown in the chart titlebar
   *
   * @param title 
   */
  void setTitle(const char* title) noexcept;
  void setTitle(const std::string& title) noexcept;
  Axis getAxisX() const noexcept;
  Axis getAxisY() const noexcept;
  LineChart(LineChart&& from) noexcept;
  LineChart& operator=(LineChart&& from) noexcept;
  ~LineChart() noexcept;
private:
  void cleanup() noexcept;
  explicit LineChart(internal::LineChartPtr cPointer) noexcept : ptr_(cPointer) {}
  internal::LineChartPtr ptr_{};
};


/**
 * @brief Represents a generic 3d object that can be rotated and translated
 */
class Object3d {
public:

  /**
   * @brief Rotates the object with a given unit quaternion
   *
   * @param qw 
   * @param qx 
   * @param qy 
   * @param qz 
   * @throw on internal errors
   */
  void setOrientation(double qw, double qx, double qy, double qz);

  /**
   * @brief Translates the object
   *
   * @param x units in [m]
   * @param y units in [m]
   * @param z units in [m]
   */
  void setTranslation(double x, double y, double z) noexcept;

  /**
   * @details 
   * Sets a 4x4 transform matrix of the form
   * 
   *     1 0 0 x
   *     0 1 0 y
   *     0 0 1 z
   *     0 0 0 1
   * 
   * The transform needs to be row-major, of size=16,
   * and include the bottom row. The translation units
   * are in meters.
   *
   * @param matrix 
   * @throw on internal errors
   */
  void setTransform4x4(const double* matrix);
  void setTransform4x4(const std::vector<double>& matrix);

  /**
   * @brief Sets visibility for this object. Hidden objects are not removed from the SceneGraph
   *
   * @param visible 
   */
  void setVisible(bool visible) noexcept;
  Object3d(Object3d&& from) noexcept;
  Object3d& operator=(Object3d&& from) noexcept;
  virtual ~Object3d() noexcept;
protected:
  explicit Object3d(internal::Object3dPtr cPointer) noexcept : ptr_(cPointer) {}
private:
  void cleanup() noexcept;
  internal::Object3dPtr ptr_{};
};


/**
 * @details 
 * Represents a line in 3d space. Note that there are currently no
 * line primitives, so the rendering is platform dependent and the
 * performance is limited.
 */
class Line3d : public Object3d {
  friend class Chart3d;
public:

  /**
   * @brief Sets the rendering color
   *
   * @param color 
   */
  void setColor(Color color) noexcept;

  /**
   * @details 
   * Sets the internal data to the given values, and sets the
   * maximum point count to the provided length.
   *
   * @param x points
   * @param y points
   * @param z points
   * @param length number of x/y/z points
   */
  void setData(const double* x, const double* y, const double* z, size_t length) noexcept;
  void setData(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) noexcept;

  /**
   * @details 
   * Sets the internal maximum point count for incrementally
   * adding points. May clear existing data.
   *
   * @param count maximum number of points
   */
  void setMaxPointCount(int count) noexcept;

  /**
   * @details 
   * Adds one point to an internal rolling buffer. Once the maximum
   * point count is reached, it will overwrite the earliest data.
   *
   * @param x 
   * @param y 
   * @param z 
   */
  void addPoint(double x, double y, double z) noexcept;
  Line3d(Line3d&& from) noexcept;
  Line3d& operator=(Line3d&& from) noexcept;
private:
  explicit Line3d(internal::Line3dPtr cPointer) noexcept;
  static internal::Object3dPtr getObject3dPointer(internal::Line3dPtr cPointer) noexcept;
  internal::Line3dPtr ptr_{};
};


/**
 * @brief Represents a static 3d mesh
 */
class Mesh3d : public Object3d {
  friend class Chart3d;
public:

  /**
   * @details 
   * Scales the loaded mesh to internal mm-units, e.g., a meter-scaled
   * mesh file would require 1e-3 to render correctly.
   *
   * @param scaleUnitsToMillimeters 
   */
  void setScale(double scaleUnitsToMillimeters) noexcept;

  /**
   * @brief Moves the origin to the center of the mesh.
   *
   * @param centered 
   */
  void setCentered(bool centered) noexcept;
  Mesh3d(Mesh3d&& from) noexcept;
  Mesh3d& operator=(Mesh3d&& from) noexcept;
private:
  explicit Mesh3d(internal::Mesh3dPtr cPointer) noexcept;
  static internal::Object3dPtr getObject3dPointer(internal::Mesh3dPtr cPointer) noexcept;
  internal::Mesh3dPtr ptr_{};
};


/**
 * @brief Represents robot kinematics
 */
class Robot3d : public Object3d {
  friend class Chart3d;
public:

  /**
   * @brief Gets the number of joints of this robot
   */
  size_t getNumJoints() noexcept;

  /**
   * @brief Renders the kinematics at the desired joint positions. The length must match the number of joints.
   *
   * @param positions 
   * @param length 
   * @throw on internal errors
   */
  void setPositions(const double* positions, size_t length);
  void setPositions(const std::vector<double>& positions);
  Robot3d(Robot3d&& from) noexcept;
  Robot3d& operator=(Robot3d&& from) noexcept;
private:
  explicit Robot3d(internal::Robot3dPtr cPointer) noexcept;
  static internal::Object3dPtr getObject3dPointer(internal::Robot3dPtr cPointer) noexcept;
  internal::Robot3dPtr ptr_{};
};


/**
 * @brief Represents a frame
 */
class Triad3d : public Object3d {
  friend class Chart3d;
public:
  Triad3d(Triad3d&& from) noexcept;
  Triad3d& operator=(Triad3d&& from) noexcept;
private:
  explicit Triad3d(internal::Triad3dPtr cPointer) noexcept;
  static internal::Object3dPtr getObject3dPointer(internal::Triad3dPtr cPointer) noexcept;
  internal::Triad3dPtr ptr_{};
};


/**
 * @details 
 * Contains utility methods for working with the library. Methods
 * are experimental and may change in the future.
 */
namespace framework {

/**
 * @brief Sets an AtlantaFX theme for rendering the UI
 *
 * @param theme 
 */
bool setTheme(Theme theme) noexcept;

/**
 * @brief Applies a global auto-close behavior, i.e., window::keepOpen
 *
 * @param autoClose 
 */
void setAutoCloseWindows(bool autoClose) noexcept;

/**
 * @brief Waits until all windows were closed by the user
 */
void waitUntilWindowsClosed() noexcept;

/**
 * @brief Debug method to run code on the internal UI thread
 *
 * @param func 
 * @param userData 
 */
void runOnUiThread(UserCallbackFunction func, void* userData) noexcept;

/**
 * @brief Debug method that prints the last exception encountered on the current thread
 */
void printLastErrorDetails() noexcept;

/**
 * @brief Debug method to print internal thread information. May be removed in the future.
 *
 * @param name 
 */
void printThreadInfo(const char* name) noexcept;
void printThreadInfo(const std::string& name) noexcept;
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
inline void Axis::setName(const char* name) noexcept {
  static auto hebi_charts_Axis_setName = DynamicLookup::instance().getFunc<void(*)(internal::AxisPtr, const char*)>("hebi_charts_Axis_setName");
  hebi_charts_Axis_setName(ptr_, name);
}
inline void Axis::setName(const std::string& name) noexcept {
  setName(name.c_str());
}
inline void Axis::setUnit(const char* unit) noexcept {
  static auto hebi_charts_Axis_setUnit = DynamicLookup::instance().getFunc<void(*)(internal::AxisPtr, const char*)>("hebi_charts_Axis_setUnit");
  hebi_charts_Axis_setUnit(ptr_, unit);
}
inline void Axis::setUnit(const std::string& unit) noexcept {
  setUnit(unit.c_str());
}
inline void Axis::setAutoUnitScaling(bool autoUnitScaling) noexcept {
  static auto hebi_charts_Axis_setAutoUnitScaling = DynamicLookup::instance().getFunc<void(*)(internal::AxisPtr, bool)>("hebi_charts_Axis_setAutoUnitScaling");
  hebi_charts_Axis_setAutoUnitScaling(ptr_, autoUnitScaling);
}
inline void Axis::setLimits(double min, double max) noexcept {
  static auto hebi_charts_Axis_setLimits = DynamicLookup::instance().getFunc<void(*)(internal::AxisPtr, double, double)>("hebi_charts_Axis_setLimits");
  hebi_charts_Axis_setLimits(ptr_, min, max);
}
inline void Axis::cleanup() noexcept {
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
inline Axis::~Axis() noexcept {
  cleanup();
}

// Chart3d
inline Robot3d Chart3d::addHrdf(const char* filePath) {
  static auto hebi_charts_Chart3d_addHrdf = DynamicLookup::instance().getFunc<internal::Robot3dPtr(*)(internal::Chart3dPtr, const char*)>("hebi_charts_Chart3d_addHrdf");
  auto ptr = hebi_charts_Chart3d_addHrdf(ptr_, filePath);
  if (!ptr) {
    throw Exception("Could not create Robot3d in Chart3d::addHrdf");
  }
  return Robot3d(ptr);
}
inline Robot3d Chart3d::addHrdf(const std::string& filePath) {
  return addHrdf(filePath.c_str());
}
inline Mesh3d Chart3d::addMesh(const char* pathOrUrl) {
  static auto hebi_charts_Chart3d_addMesh = DynamicLookup::instance().getFunc<internal::Mesh3dPtr(*)(internal::Chart3dPtr, const char*)>("hebi_charts_Chart3d_addMesh");
  auto ptr = hebi_charts_Chart3d_addMesh(ptr_, pathOrUrl);
  if (!ptr) {
    throw Exception("Could not create Mesh3d in Chart3d::addMesh");
  }
  return Mesh3d(ptr);
}
inline Mesh3d Chart3d::addMesh(const std::string& pathOrUrl) {
  return addMesh(pathOrUrl.c_str());
}
inline Triad3d Chart3d::addTriad(double lengthInMeters) {
  static auto hebi_charts_Chart3d_addTriad = DynamicLookup::instance().getFunc<internal::Triad3dPtr(*)(internal::Chart3dPtr, double)>("hebi_charts_Chart3d_addTriad");
  auto ptr = hebi_charts_Chart3d_addTriad(ptr_, lengthInMeters);
  if (!ptr) {
    throw Exception("Could not create Triad3d in Chart3d::addTriad");
  }
  return Triad3d(ptr);
}
inline Line3d Chart3d::addLine() {
  static auto hebi_charts_Chart3d_addLine = DynamicLookup::instance().getFunc<internal::Line3dPtr(*)(internal::Chart3dPtr)>("hebi_charts_Chart3d_addLine");
  auto ptr = hebi_charts_Chart3d_addLine(ptr_);
  if (!ptr) {
    throw Exception("Could not create Line3d in Chart3d::addLine");
  }
  return Line3d(ptr);
}
inline void Chart3d::cleanup() noexcept {
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
inline Chart3d::~Chart3d() noexcept {
  cleanup();
}

// GridWindow
inline GridWindow::GridWindow(int rows, int cols) {
  static auto hebi_charts_GridWindow_create = DynamicLookup::instance().getFunc<internal::GridWindowPtr(*)(int, int)>("hebi_charts_GridWindow_create");
  ptr_ = hebi_charts_GridWindow_create(rows, cols);
  if (!ptr_) {
    throw Exception("Could not create GridWindow");
  }
}
inline LineChart GridWindow::addLineChart(int row, int col, int rowSpan, int colSpan) {
  static auto hebi_charts_GridWindow_addLineChart = DynamicLookup::instance().getFunc<internal::LineChartPtr(*)(internal::GridWindowPtr, int, int, int, int)>("hebi_charts_GridWindow_addLineChart");
  auto ptr = hebi_charts_GridWindow_addLineChart(ptr_, row, col, rowSpan, colSpan);
  if (!ptr) {
    throw Exception("Could not create LineChart in GridWindow::addLineChart");
  }
  return LineChart(ptr);
}
inline Chart3d GridWindow::addChart3d(int row, int col, int rowSpan, int colSpan) {
  static auto hebi_charts_GridWindow_addChart3d = DynamicLookup::instance().getFunc<internal::Chart3dPtr(*)(internal::GridWindowPtr, int, int, int, int)>("hebi_charts_GridWindow_addChart3d");
  auto ptr = hebi_charts_GridWindow_addChart3d(ptr_, row, col, rowSpan, colSpan);
  if (!ptr) {
    throw Exception("Could not create Chart3d in GridWindow::addChart3d");
  }
  return Chart3d(ptr);
}
inline void GridWindow::setSize(int width, int height) noexcept {
  static auto hebi_charts_GridWindow_setSize = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr, int, int)>("hebi_charts_GridWindow_setSize");
  hebi_charts_GridWindow_setSize(ptr_, width, height);
}
inline void GridWindow::setLocation(int xOffset, int yOffset) noexcept {
  static auto hebi_charts_GridWindow_setLocation = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr, int, int)>("hebi_charts_GridWindow_setLocation");
  hebi_charts_GridWindow_setLocation(ptr_, xOffset, yOffset);
}
inline void GridWindow::setTitle(const char* title) noexcept {
  static auto hebi_charts_GridWindow_setTitle = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr, const char*)>("hebi_charts_GridWindow_setTitle");
  hebi_charts_GridWindow_setTitle(ptr_, title);
}
inline void GridWindow::setTitle(const std::string& title) noexcept {
  setTitle(title.c_str());
}
inline void GridWindow::show() noexcept {
  static auto hebi_charts_GridWindow_show = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr)>("hebi_charts_GridWindow_show");
  hebi_charts_GridWindow_show(ptr_);
}
inline void GridWindow::hide() noexcept {
  static auto hebi_charts_GridWindow_hide = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr)>("hebi_charts_GridWindow_hide");
  hebi_charts_GridWindow_hide(ptr_);
}
inline bool GridWindow::isShowing() const noexcept {
  static auto hebi_charts_GridWindow_isShowing = DynamicLookup::instance().getFunc<bool(*)(internal::GridWindowPtr)>("hebi_charts_GridWindow_isShowing");
  return hebi_charts_GridWindow_isShowing(ptr_);
}
inline void GridWindow::keepOpen(bool keepOpen) noexcept {
  static auto hebi_charts_GridWindow_keepOpen = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr, bool)>("hebi_charts_GridWindow_keepOpen");
  hebi_charts_GridWindow_keepOpen(ptr_, keepOpen);
}
inline void GridWindow::waitUntilClosed() const noexcept {
  static auto hebi_charts_GridWindow_waitUntilClosed = DynamicLookup::instance().getFunc<void(*)(internal::GridWindowPtr)>("hebi_charts_GridWindow_waitUntilClosed");
  hebi_charts_GridWindow_waitUntilClosed(ptr_);
}
inline void GridWindow::cleanup() noexcept {
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
inline GridWindow::~GridWindow() noexcept {
  cleanup();
}

// Line
inline void Line::setColor(Color color) noexcept {
  static auto hebi_charts_Line_setColor = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, Color)>("hebi_charts_Line_setColor");
  hebi_charts_Line_setColor(ptr_, color);
}
inline void Line::setLineStyle(LineStyle lineStyle) noexcept {
  static auto hebi_charts_Line_setLineStyle = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, LineStyle)>("hebi_charts_Line_setLineStyle");
  hebi_charts_Line_setLineStyle(ptr_, lineStyle);
}
inline void Line::setData(const double* x, const double* y, size_t length) noexcept {
  static auto hebi_charts_Line_setData = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, const double*, const double*, size_t)>("hebi_charts_Line_setData");
  hebi_charts_Line_setData(ptr_, x, y, length);
}
inline void Line::setData(const std::vector<double>& x, const std::vector<double>& y) noexcept {
  setData(x.data(), y.data(), (std::min)(x.size(), y.size()));
}
inline void Line::setMaxPointCount(int count) noexcept {
  static auto hebi_charts_Line_setMaxPointCount = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, int)>("hebi_charts_Line_setMaxPointCount");
  hebi_charts_Line_setMaxPointCount(ptr_, count);
}
inline void Line::addPoint(double x, double y) noexcept {
  static auto hebi_charts_Line_addPoint = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, double, double)>("hebi_charts_Line_addPoint");
  hebi_charts_Line_addPoint(ptr_, x, y);
}
inline void Line::setName(const char* name) noexcept {
  static auto hebi_charts_Line_setName = DynamicLookup::instance().getFunc<void(*)(internal::LinePtr, const char*)>("hebi_charts_Line_setName");
  hebi_charts_Line_setName(ptr_, name);
}
inline void Line::setName(const std::string& name) noexcept {
  setName(name.c_str());
}
inline void Line::cleanup() noexcept {
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
inline Line::~Line() noexcept {
  cleanup();
}

// LineChart
inline Line LineChart::addLine(const char* name, const double* x, const double* y, size_t length) {
  static auto hebi_charts_LineChart_addLine = DynamicLookup::instance().getFunc<internal::LinePtr(*)(internal::LineChartPtr, const char*, const double*, const double*, size_t)>("hebi_charts_LineChart_addLine");
  auto ptr = hebi_charts_LineChart_addLine(ptr_, name, x, y, length);
  if (!ptr) {
    throw Exception("Could not create Line in LineChart::addLine");
  }
  return Line(ptr);
}
inline Line LineChart::addLine(const std::string& name) {
  return addLine(name.c_str(), nullptr, nullptr, 0);
}
inline Line LineChart::addLine(const std::string& name, const std::vector<double>& x, const std::vector<double>& y) {
  return addLine(name.c_str(), x.data(), y.data(), (std::min)(x.size(), y.size()));
}
inline void LineChart::setTitle(const char* title) noexcept {
  static auto hebi_charts_LineChart_setTitle = DynamicLookup::instance().getFunc<void(*)(internal::LineChartPtr, const char*)>("hebi_charts_LineChart_setTitle");
  hebi_charts_LineChart_setTitle(ptr_, title);
}
inline void LineChart::setTitle(const std::string& title) noexcept {
  setTitle(title.c_str());
}
inline Axis LineChart::getAxisX() const noexcept {
  static auto hebi_charts_LineChart_getAxisX = DynamicLookup::instance().getFunc<internal::AxisPtr(*)(internal::LineChartPtr)>("hebi_charts_LineChart_getAxisX");
  auto ptr = hebi_charts_LineChart_getAxisX(ptr_);
  return Axis(ptr);
}
inline Axis LineChart::getAxisY() const noexcept {
  static auto hebi_charts_LineChart_getAxisY = DynamicLookup::instance().getFunc<internal::AxisPtr(*)(internal::LineChartPtr)>("hebi_charts_LineChart_getAxisY");
  auto ptr = hebi_charts_LineChart_getAxisY(ptr_);
  return Axis(ptr);
}
inline void LineChart::cleanup() noexcept {
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
inline LineChart::~LineChart() noexcept {
  cleanup();
}

// Object3d
inline void Object3d::setOrientation(double qw, double qx, double qy, double qz) {
  static auto hebi_charts_Object3d_setOrientation = DynamicLookup::instance().getFunc<void(*)(internal::Object3dPtr, double, double, double, double)>("hebi_charts_Object3d_setOrientation");
  hebi_charts_Object3d_setOrientation(ptr_, qw, qx, qy, qz);
}
inline void Object3d::setTranslation(double x, double y, double z) noexcept {
  static auto hebi_charts_Object3d_setTranslation = DynamicLookup::instance().getFunc<void(*)(internal::Object3dPtr, double, double, double)>("hebi_charts_Object3d_setTranslation");
  hebi_charts_Object3d_setTranslation(ptr_, x, y, z);
}
inline void Object3d::setTransform4x4(const double* matrix) {
  static auto hebi_charts_Object3d_setTransform4x4 = DynamicLookup::instance().getFunc<void(*)(internal::Object3dPtr, const double*)>("hebi_charts_Object3d_setTransform4x4");
  hebi_charts_Object3d_setTransform4x4(ptr_, matrix);
}
inline void Object3d::setTransform4x4(const std::vector<double>& matrix) {
  if (matrix.size() != 16) {
    throw Exception("Transform matrix incorrect size");
  }
  setTransform4x4(matrix.data());
}
inline void Object3d::setVisible(bool visible) noexcept {
  static auto hebi_charts_Object3d_setVisible = DynamicLookup::instance().getFunc<void(*)(internal::Object3dPtr, bool)>("hebi_charts_Object3d_setVisible");
  hebi_charts_Object3d_setVisible(ptr_, visible);
}
inline void Object3d::cleanup() noexcept {
  if (ptr_ != nullptr) {
    static auto hebi_charts_Object3d_release = DynamicLookup::instance().getFunc<void(*)(internal::Object3dPtr)>("hebi_charts_Object3d_release");
    hebi_charts_Object3d_release(ptr_);
  }
}
inline Object3d::Object3d(Object3d&& from) noexcept : ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Object3d& Object3d::operator=(Object3d&& from) noexcept {
  cleanup();
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};
inline Object3d::~Object3d() noexcept {
  cleanup();
}

// Line3d
inline void Line3d::setColor(Color color) noexcept {
  static auto hebi_charts_Line3d_setColor = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, Color)>("hebi_charts_Line3d_setColor");
  hebi_charts_Line3d_setColor(ptr_, color);
}
inline void Line3d::setData(const double* x, const double* y, const double* z, size_t length) noexcept {
  static auto hebi_charts_Line3d_setData = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, const double*, const double*, const double*, size_t)>("hebi_charts_Line3d_setData");
  hebi_charts_Line3d_setData(ptr_, x, y, z, length);
}
inline void Line3d::setData(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) noexcept {
  setData(x.data(), y.data(), z.data(), (std::min)({x.size(), y.size(), z.size()}));
}
inline void Line3d::setMaxPointCount(int count) noexcept {
  static auto hebi_charts_Line3d_setMaxPointCount = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, int)>("hebi_charts_Line3d_setMaxPointCount");
  hebi_charts_Line3d_setMaxPointCount(ptr_, count);
}
inline void Line3d::addPoint(double x, double y, double z) noexcept {
  static auto hebi_charts_Line3d_addPoint = DynamicLookup::instance().getFunc<void(*)(internal::Line3dPtr, double, double, double)>("hebi_charts_Line3d_addPoint");
  hebi_charts_Line3d_addPoint(ptr_, x, y, z);
}
inline internal::Object3dPtr Line3d::getObject3dPointer(internal::Line3dPtr cPointer) noexcept {
  static auto hebi_charts_Line3d_to_Object3d = DynamicLookup::instance().getFunc<internal::Object3dPtr(*)(internal::Line3dPtr)>("hebi_charts_Line3d_to_Object3d");
  return hebi_charts_Line3d_to_Object3d(cPointer);
}
inline Line3d::Line3d(internal::Line3dPtr cPointer) noexcept : Object3d(getObject3dPointer(cPointer)), ptr_(cPointer) {}
inline Line3d::Line3d(Line3d&& from) noexcept : Object3d(std::move(from)), ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Line3d& Line3d::operator=(Line3d&& from) noexcept {
  Object3d::operator=(std::move(from));
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};

// Mesh3d
inline void Mesh3d::setScale(double scaleUnitsToMillimeters) noexcept {
  static auto hebi_charts_Mesh3d_setScale = DynamicLookup::instance().getFunc<void(*)(internal::Mesh3dPtr, double)>("hebi_charts_Mesh3d_setScale");
  hebi_charts_Mesh3d_setScale(ptr_, scaleUnitsToMillimeters);
}
inline void Mesh3d::setCentered(bool centered) noexcept {
  static auto hebi_charts_Mesh3d_setCentered = DynamicLookup::instance().getFunc<void(*)(internal::Mesh3dPtr, bool)>("hebi_charts_Mesh3d_setCentered");
  hebi_charts_Mesh3d_setCentered(ptr_, centered);
}
inline internal::Object3dPtr Mesh3d::getObject3dPointer(internal::Mesh3dPtr cPointer) noexcept {
  static auto hebi_charts_Mesh3d_to_Object3d = DynamicLookup::instance().getFunc<internal::Object3dPtr(*)(internal::Mesh3dPtr)>("hebi_charts_Mesh3d_to_Object3d");
  return hebi_charts_Mesh3d_to_Object3d(cPointer);
}
inline Mesh3d::Mesh3d(internal::Mesh3dPtr cPointer) noexcept : Object3d(getObject3dPointer(cPointer)), ptr_(cPointer) {}
inline Mesh3d::Mesh3d(Mesh3d&& from) noexcept : Object3d(std::move(from)), ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Mesh3d& Mesh3d::operator=(Mesh3d&& from) noexcept {
  Object3d::operator=(std::move(from));
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};

// Robot3d
inline size_t Robot3d::getNumJoints() noexcept {
  static auto hebi_charts_Robot3d_getNumJoints = DynamicLookup::instance().getFunc<size_t(*)(internal::Robot3dPtr)>("hebi_charts_Robot3d_getNumJoints");
  return hebi_charts_Robot3d_getNumJoints(ptr_);
}
inline void Robot3d::setPositions(const double* positions, size_t length) {
  static auto hebi_charts_Robot3d_setPositions = DynamicLookup::instance().getFunc<void(*)(internal::Robot3dPtr, const double*, size_t)>("hebi_charts_Robot3d_setPositions");
  if (length != getNumJoints())
    throw Exception("Position vector length does not match number of joints");
  hebi_charts_Robot3d_setPositions(ptr_, positions, length);
}
inline void Robot3d::setPositions(const std::vector<double>& positions) {
  setPositions(positions.data(), positions.size());
}
inline internal::Object3dPtr Robot3d::getObject3dPointer(internal::Robot3dPtr cPointer) noexcept {
  static auto hebi_charts_Robot3d_to_Object3d = DynamicLookup::instance().getFunc<internal::Object3dPtr(*)(internal::Robot3dPtr)>("hebi_charts_Robot3d_to_Object3d");
  return hebi_charts_Robot3d_to_Object3d(cPointer);
}
inline Robot3d::Robot3d(internal::Robot3dPtr cPointer) noexcept : Object3d(getObject3dPointer(cPointer)), ptr_(cPointer) {}
inline Robot3d::Robot3d(Robot3d&& from) noexcept : Object3d(std::move(from)), ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Robot3d& Robot3d::operator=(Robot3d&& from) noexcept {
  Object3d::operator=(std::move(from));
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};

// Triad3d
inline internal::Object3dPtr Triad3d::getObject3dPointer(internal::Triad3dPtr cPointer) noexcept {
  static auto hebi_charts_Triad3d_to_Object3d = DynamicLookup::instance().getFunc<internal::Object3dPtr(*)(internal::Triad3dPtr)>("hebi_charts_Triad3d_to_Object3d");
  return hebi_charts_Triad3d_to_Object3d(cPointer);
}
inline Triad3d::Triad3d(internal::Triad3dPtr cPointer) noexcept : Object3d(getObject3dPointer(cPointer)), ptr_(cPointer) {}
inline Triad3d::Triad3d(Triad3d&& from) noexcept : Object3d(std::move(from)), ptr_(from.ptr_) {
  from.ptr_ = nullptr;
};
inline Triad3d& Triad3d::operator=(Triad3d&& from) noexcept {
  Object3d::operator=(std::move(from));
  ptr_ = from.ptr_;
  from.ptr_ = nullptr;
  return *this;
};

// Framework
inline bool framework::setTheme(Theme theme) noexcept {
  static auto hebi_charts_Framework_setTheme = DynamicLookup::instance().getFunc<bool(*)(Theme)>("hebi_charts_Framework_setTheme");
  return hebi_charts_Framework_setTheme(theme);
}
inline void framework::setAutoCloseWindows(bool autoClose) noexcept {
  static auto hebi_charts_Framework_setAutoCloseWindows = DynamicLookup::instance().getFunc<void(*)(bool)>("hebi_charts_Framework_setAutoCloseWindows");
  hebi_charts_Framework_setAutoCloseWindows(autoClose);
}
inline void framework::waitUntilWindowsClosed() noexcept {
  static auto hebi_charts_Framework_waitUntilWindowsClosed = DynamicLookup::instance().getFunc<void(*)()>("hebi_charts_Framework_waitUntilWindowsClosed");
  hebi_charts_Framework_waitUntilWindowsClosed();
}
inline void framework::runOnUiThread(UserCallbackFunction func, void* userData) noexcept {
  static auto hebi_charts_Framework_runOnUiThread = DynamicLookup::instance().getFunc<void(*)(UserCallbackFunction, void*)>("hebi_charts_Framework_runOnUiThread");
  hebi_charts_Framework_runOnUiThread(func, userData);
}
inline void framework::printLastErrorDetails() noexcept {
  static auto hebi_charts_Framework_printLastErrorDetails = DynamicLookup::instance().getFunc<void(*)()>("hebi_charts_Framework_printLastErrorDetails");
  hebi_charts_Framework_printLastErrorDetails();
}
inline void framework::printThreadInfo(const char* name) noexcept {
  static auto hebi_charts_Framework_printThreadInfo = DynamicLookup::instance().getFunc<void(*)(const char*)>("hebi_charts_Framework_printThreadInfo");
  hebi_charts_Framework_printThreadInfo(name);
}
inline void framework::printThreadInfo(const std::string& name) noexcept {
  printThreadInfo(name.c_str());
}

// ==== Cocoa utilities for supporting macOS ====
typedef int (*hebi_charts_MainCallbackFunction)(int argc, char** argv);
/**
 * @details 
 * Sets up required system libraries and executes the callback on an appropriate thread
 * 
 * This is technically only needed on macOS as the Cocoa framework for displaying
 * windows needs to be run on the main thread. On Windows and Linux this method
 * executes the callback directly and otherwise does nothing. However, all platforms
 * are supported to enable platform-independent code with the same behavior.
 */
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
  return {0, 5, 0, 66};
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
