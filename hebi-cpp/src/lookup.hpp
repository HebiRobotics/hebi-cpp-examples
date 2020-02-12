#pragma once

#include "hebi.h"

#include <cstddef>
#include <iterator>
#include <memory> // For shared_ptr
#include <vector>

#include "group.hpp"
#include "mac_address.hpp"

namespace hebi {

/**
 * \brief Maintains a registry of network-connected modules and returns Group
 * objects to the user.
 *
 * Only one Lookup object is needed per application.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 18 Feb 2016
 */
class Lookup final {
private:
  static const int32_t DEFAULT_TIMEOUT = 500;

  /**
   * \internal C-style lookup object
   */
  HebiLookupPtr lookup_;

  /**
   * Default value of feedback frequency used when creating new groups.
   */
  float initial_group_feedback_frequency_{100.0f};

  /**
   * Default value of command lifetime used when creating new groups.
   */
  int32_t initial_group_command_lifetime_{250};

public:
  /**
   * \brief Creates a Lookup object which can create Module and Group
   * references.
   * Typically, only one Lookup object should exist at a time.
   *
   * Note that this call invokes a background thread to query the network for
   * modules at regular intervals.
   */
  Lookup();

  /**
   * \brief Destructor frees all resources created by Lookup object, and stops the
   * background query thread.
   */
  ~Lookup() noexcept; /* annotating specified destructor as noexcept is best-practice */

  /**
   * \brief Get a group from modules with the given names and families.
   *
   * If one of the input vectors is of length one, then that element is
   * assumed to pair with each item in the other input vector.
   *
   * Blocking call which returns a reference to a Group object with the given
   * parameters. Times out after timeout_msec milliseconds.
   *
   * @param families A list of families of desired group modules, as viewable
   * in the HEBI GUI.  If of length one, this family is paried with each given
   * name.
   * @param names A list of names of desired group modules, as viewable in the
   * HEBI GUI.  If of length one, this name is paired with each given family
   * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
   * a group is found, and a value of 0 returns immediately if no group with
   * that address is currently known by the Lookup class.
   * @returns A shared_ptr with no reference if no group found in allotted
   * time, or reference to a newly allocated group object corresponding to
   * the given parameters otherwise.
   */
  std::shared_ptr<Group> getGroupFromNames(const std::vector<std::string>& families,
                                           const std::vector<std::string>& names, int32_t timeout_ms = DEFAULT_TIMEOUT);

  /**
   * \brief Get a group from modules with the given mac addresses.
   *
   * Blocking call which returns a reference to a Group object with the given
   * parameters. Times out after timeout_msec milliseconds.
   *
   * @param addresses List of physical mac addresses for desired group modules.
   * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
   * a group is found, and a value of 0 returns immediately if no group with
   * that address is currently known by the Lookup class.
   * @returns A shared_ptr with no reference if no group found in allotted
   * time, or reference to a newly allocated group object corresponding to
   * the given parameters otherwise.
   */
  std::shared_ptr<Group> getGroupFromMacs(const std::vector<MacAddress>& addresses,
                                          int32_t timeout_ms = DEFAULT_TIMEOUT);

  /**
   * \brief Get a group from all known modules with the given family.
   *
   * Blocking call which returns a reference to a Group object with the given
   * parameters. Times out after timeout_msec milliseconds.
   *
   * @param family The family of each of the desired group modules.
   * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
   * a group is found, and a value of 0 returns immediately if no group with
   * that address is currently known by the Lookup class.
   * @returns A shared_ptr with no reference if no group found in allotted
   * time, or reference to a newly allocated group object corresponding to
   * the given parameters otherwise.
   */
  std::shared_ptr<Group> getGroupFromFamily(const std::string& family, int32_t timeout_ms = DEFAULT_TIMEOUT);

  /**
   * \brief Get a group from all modules known to connect to a module with the
   * given name and family.
   *
   * Blocking call which returns a reference to a Group object with the given
   * parameters. Times out after timeout_msec milliseconds.
   *
   * @param family The given family of the module, as viewable in the HEBI
   * GUI, to form the group from.
   * @param name The given name of the module, as viewable in the HEBI GUI, to
   * form the group from.
   * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
   * a group is found, and a value of 0 returns immediately if no group with
   * that address is currently known by the Lookup class.
   * @returns A shared_ptr with no reference if no group found in allotted
   * time, or reference to a newly allocated group object corresponding to
   * the given parameters otherwise.
   */
  std::shared_ptr<Group> getConnectedGroupFromName(const std::string& family, const std::string& name,
                                                   int32_t timeout_ms = DEFAULT_TIMEOUT);

  /**
   * \brief Get a group from all modules known to connect to a module with the
   * given mac address.
   *
   * Blocking call which returns a reference to a Group object with the given
   * parameters. Times out after timeout_msec milliseconds.
   *
   * @param address Physical mac address of the module to form the group from.
   * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
   * a group is found, and a value of 0 returns immediately if no group with
   * that address is currently known by the Lookup class.
   * @returns A shared_ptr with no reference if no group found in allotted
   * time, or reference to a newly allocated group object corresponding to
   * the given parameters otherwise.
   */
  std::shared_ptr<Group> getConnectedGroupFromMac(const MacAddress& address, int32_t timeout_ms = DEFAULT_TIMEOUT);

  /**
   * \brief Gets the default feedback frequency value for groups created from
   * this lookup.
   *
   * Defaults to 100 Hz.
   *
   * See Group documentation for detailed usage, and docs.hebi.us for more
   * information on feedback requests.
   */
  float getInitialGroupFeedbackFrequencyHz();
  /**
   * \brief Sets the default feedback frequency value for groups created from
   * this lookup.
   *
   * @param frequency The frequency, in Hz, that newly-created groups will
   * request feedback at.
   */
  void setInitialGroupFeedbackFrequencyHz(float frequency);

  /**
   * \brief Gets the default command lifetime value for groups created from
   * this lookup.
   *
   * Defaults to 250 ms.
   *
   * See Group documentation for detailed usage, and docs.hebi.us for more
   * information on Command Lifetime.
   */
  int32_t getInitialGroupCommandLifetimeMs();
  /**
   * \brief Sets the default command lifetime value for groups created from
   * this lookup.
   *
   * @param ms The number of milliseconds that newly-created groups will use
   * as their command lifetime when sending out commands.
   */
  void setInitialGroupCommandLifetimeMs(int32_t ms);

  /**
   * \brief Gets the rate [Hz] at which "discovery" packets are broadcast.
   *
   * Defaults to 5 Hz.
   */
  double getLookupFrequencyHz() const;

  /**
   * \brief Sets the lookup rate [Hz]
   *
   * @param frequency The rate at which "discovery" packets get broadcast on
   * the network to search for modules.
   *
   * \returns true on success, false on failure (e.g., invalid frequency)
   */
  bool setLookupFrequencyHz(double frequency);

  class EntryList final {
    struct Entry final {
      std::string name_;
      std::string family_;
      MacAddress mac_address_;
    };

  private:
    /**
     * \internal C-style lookup entry list object
     */
    HebiLookupEntryListPtr lookup_list_;

    /**
     * \internal Entry list iterator implementation
     * (see http://anderberg.me/2016/07/04/c-custom-iterators/)
     */
    class Iterator final {
    public:
      // Iterator traits (not from std::iterator to be C++17 compliant)
      using value_type = Entry;
      using difference_type = int;
      using pointer = Entry*;
      using reference = Entry;
      using iterator_category = std::bidirectional_iterator_tag;

      // Default constructable
      Iterator() = default;
      explicit Iterator(const EntryList& list, size_t current);

      // Dereferencable
      reference operator*() const;

      // Pre- and post-incrementable/decrementable
      Iterator& operator++();
      Iterator operator++(int);
      Iterator& operator--();
      Iterator operator--(int);

      // Equality / inequality
      bool operator==(const Iterator& rhs) const;
      bool operator!=(const Iterator& rhs) const;

    private:
      const EntryList& list_;
      size_t current_{0};
    };

  public:
    /**
     * \internal Creates entry list from internal C-style object.
     */
    EntryList(HebiLookupEntryListPtr lookup_list) : lookup_list_(lookup_list) {}

    ~EntryList() noexcept;

    Entry operator[](size_t index) const;

    size_t size() const;

    Iterator begin() const;
    Iterator end() const;

  private:
    /**
     * Disable copy and move constructors and assignment operators
     */
    HEBI_DISABLE_COPY_MOVE(EntryList)
  };

  std::shared_ptr<EntryList> getEntryList();

private:
  /**
   * Disable copy and move constructors and assignment operators
   */
  HEBI_DISABLE_COPY_MOVE(Lookup)
};

} // namespace hebi
