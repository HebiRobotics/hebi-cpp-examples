#pragma once

#include <string>

// clang-format off
#define HEBI_DISABLE_COPY_MOVE(Class) \
/* Disable copy constructor. */ \
Class(const Class& other) = delete; \
/* Disable move constructor. */ \
Class(Class&& other) = delete; \
/* Disable copy assigment operator. */ \
Class& operator= (const Class& other) = delete; \
/* Disable move assigment operator. */ \
Class& operator= (Class&& other) = delete;

#define HEBI_DISABLE_COPY(Class) \
/* Disable copy constructor. */ \
Class(const Class& other) = delete; \
/* Disable copy assigment operator. */ \
Class& operator= (const Class& other) = delete;
// clang-format on

namespace hebi {

/**
 * @brief Used as a return 
 */
class FunctionCallResult {
public:
  explicit FunctionCallResult(bool success) : ok_(success), failure_message_("") {}
  FunctionCallResult(bool success, const std::string& failure) : ok_(success), failure_message_(failure) {}
  FunctionCallResult(FunctionCallResult&&) = default;
  FunctionCallResult& operator=(FunctionCallResult&&) = default;

  operator bool() const { return ok_; }
  bool success() const { return ok_; }

  /**
   * The failure message, if the call this represents failed.
   * 
   * On success, this will always be a valid empty string (_i.e._,  @c result.failureMessage().empty() is always true).
   */
  const std::string& failureMessage() const { return failure_message_; }
private:
  bool ok_;
  std::string failure_message_;
};

}
