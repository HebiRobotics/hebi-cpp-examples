/**
 * @file lookup_helpers.cpp common helper functions used by various examples.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 22 Jan 2016
 */

// HEBI classes
#include "lookup.hpp"
#include "group.hpp"
#include "mac_address.hpp"

// For help parsing command line options
#include "optionparser.h"

#include <iostream>

struct Arg : public option::Arg
{
  static void printError(const char* msg1, const option::Option& opt, const char* msg2)
  {
    fprintf(stderr, "%s", msg1);
    fwrite(opt.name, opt.namelen, 1, stderr);
    fprintf(stderr, "%s", msg2);
  }

  static option::ArgStatus Unknown(const option::Option& option, bool msg)
  {
    if (msg) printError("Unknown option '", option, "'\n");
    return option::ARG_ILLEGAL;
  }

  static option::ArgStatus Pair(const option::Option& option, bool msg)
  {
    if (option.arg != nullptr)
    {
      // Parse name arguments (family|name)
      std::string pair_string = std::string(option.arg);
      auto split_point = pair_string.find('|');
      if (split_point != std::string::npos &&    // has split character
          split_point != 0 &&                    // has family
          split_point != pair_string.size() - 1) // has name
        return option::ARG_OK;
    }
   
    if (msg) printError("Option '", option, "' must be in <family>|<name> format. Note that you may need to escape the '|' character\n");
    return option::ARG_ILLEGAL;
  }

  static option::ArgStatus Mac(const option::Option& option, bool msg)
  {
    if (option.arg != nullptr && hebi::MacAddress::isHexStringValid(option.arg))
      return option::ARG_OK;

    if (msg) printError("Option '", option, "' must be in 00:01:02:ab:cd:ef format.\n");
    return option::ARG_ILLEGAL;
  }

  static option::ArgStatus NonEmpty(const option::Option& option, bool msg)
  {
    if (option.arg != nullptr && option.arg[0] != 0)
      return option::ARG_OK;

    if (msg) printError("Option '", option, "' must be a string.\n");
    return option::ARG_ILLEGAL;
  }
};

enum optionIndex { UNKNOWN, HELP, VERBOSE, NAME, FAMILY, PAIR, MAC, NAMES, FAMILIES, PAIRS, MACS };

const option::Descriptor groupUsage[] =
{
  { UNKNOWN, 0, "", "",            Arg::Unknown, "USAGE: program_name [options]\n\n"
    "Options are parsed to determine search terms for group. User can either provide:\n"
    " 1) single module specification to form connected group (-p, -n/-f, or -m)\n"
    " 2) list of families and list of names independently (-F and -N)\n"
    " 3) list of family/name pairs (-P)\n"
    " 4) list of mac addresses (-M)\n\n"
    "Options:" },
  { HELP,    0, "h", "help",       option::Arg::None, "  -h, \t--help  \tPrint this help and exit." },
  { VERBOSE, 0, "v", "verbose",    option::Arg::None, "  -v, \t--verbose  \tVerbose output is enabled to help diagnose lookup issues." },
  { NAME,    0, "n", "name",       Arg::NonEmpty, "  -n <name>, \t--name=<name> \tName of the module; only for use in combination with '-f'. Used to create a group from all connected modules." },
  { FAMILY,  0, "f", "family",     Arg::NonEmpty, "  -f <family_name>, \t--family=<family_name> \tName of the family; only for use in combination with '-n'. Used to create a group from all connected modules." },
  { PAIR,    0, "p", "pair",       Arg::Pair, "  -p <name pair>, \t--pair=<name pair> \tFamily + name pair; must write as family_name|name, with '|' symbol between family and name.  (Note: may need to escape the '|' character in some shells). Creates a group from all connected modules." },
  { MAC,     0, "m", "mac",        Arg::Mac, "  -m <mac>, \t--mac=<mac> \tMAC address of the module; must be given in following format: 01:23:45:ab:cd:ef.  Creates a group from all connected modules." },
  { NAMES,    0, "N", "names",       Arg::NonEmpty, "  -N <name> \t--names=<name> \tUsed to define a set of module names; each '-N' or '--names' argument is another module.  Only for use in combination with '-F'." },
  { FAMILIES,  0, "F", "families",     Arg::NonEmpty, "  -F <family_name>, \t--families=<family_name> \tUsed to define a set of family names; each '-F' or '--families' argument is another family. Only for use in combination with '-N'. Each -F/--families argument is combined pairwise with each -N/--names argument.  If only one '-F' is given for a list of '-N' arguments, it applies to each '-N'." },
  { PAIRS,    0, "P", "pairs",       Arg::Pair, "  -P <name pair>, \t--pairs=<name pair> \tUsed to define a set of family + name pairs; each '-P' or '--pairs' argument is another pair. Matches format of '-p' argument." },
  { MACS,     0, "M", "macs",        Arg::Mac, "  -M <mac>, \t--macs=<mac> \tUsed to define a set of MAC addresses; each '-M' argument is another mac address. Matches format of '-m' argument." },
  { UNKNOWN, 0, "", "", option::Arg::None,
 "\nExamples:\n"
 "  ./script -nModule05 -fArm \n"
 "  ./script -pArm|Module05 \n"
 "  ./script -ma1:2f:45:a3:c9:ef \n"
 "  ./script -NModule05 -NModule07 -NModule02 -FArm -FArm -FArm \n"
 "  ./script -N Module05 -N Module07 -N Module02 -F Arm \n"
 "  ./script -P Arm|Module05 -P Arm|Module06 \n"
 "  ./script -M 01:ab:23:4b:cd:4f --macs 0f:a4:25:a4:fe:3d \n" },
{ 0, 0, 0, 0, 0, 0 } };

/**
 * Parse the given arguments, and look up group based on the given arguments.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 22 Jan 2016
 */
std::shared_ptr<hebi::Group> getGroupFromArgs(int argc, char* argv[])
{
  // Parse command line arguments into an options struct.
  argc-=(argc>0); argv+=(argc>0); // skip program name argv[0] if present
  option::Stats stats(groupUsage, argc, argv);
  std::vector<option::Option> options(stats.options_max);
  std::vector<option::Option> buffer(stats.buffer_max);
  option::Parser parse(groupUsage, argc, argv, &options[0], &buffer[0], 0, true);

  if (parse.error())
    return nullptr;

  // Display help and exit
  if (options[HELP] || argc == 0)
  {
    option::printUsage(std::cout, groupUsage);
    return nullptr;
  }

  // Verify options are enough to run a lookup
  bool printModuleList = options[VERBOSE];
  option::Option* pair_opt = options[PAIR];
  option::Option* name_opt = options[NAME];
  option::Option* family_opt = options[FAMILY];
  option::Option* mac_opt = options[MAC];
  option::Option* pairs_opt = options[PAIRS];
  option::Option* names_opt = options[NAMES];
  int names_count = options[NAMES].count();
  option::Option* families_opt = options[FAMILIES];
  int families_count = options[FAMILIES].count();
  option::Option* macs_opt = options[MACS];
  if (!pair_opt && (!name_opt || !family_opt) && !mac_opt && // connected group cases
      !pairs_opt && (!names_opt || !families_opt || (names_count != families_count && names_count != 1 && families_count != 1)) && !macs_opt) // explicit list of modules cases
  {
    printf("Invalid combination of arguments!\n"); // TODO: can we detect with parser?
    return nullptr;
  }

  // Setup the lookup
  long timeout_ms = 4000; // Give the modules plenty of time to appear.
  hebi::Lookup lookup;
  std::shared_ptr<hebi::Group> group;

  // Query the lookup depending on arguments
  // (note: argument formats have already been validated above!)
  if (pair_opt)
  {
    std::string pair_string = std::string(pair_opt->arg);
    auto split_point = pair_string.find('|');
    group = lookup.getConnectedGroupFromName(pair_string.substr(split_point + 1), pair_string.substr(0, split_point), timeout_ms);
  }
  else if (name_opt && family_opt)
  {
    group = lookup.getConnectedGroupFromName(std::string(name_opt->arg), std::string(family_opt->arg), timeout_ms);
  }
  else if (mac_opt)
  {
    hebi::MacAddress mac;
    mac.setToHexString(std::string(mac_opt->arg));
    group = lookup.getConnectedGroupFromMac(mac, timeout_ms);
  }
  else if (pairs_opt)
  {
    std::vector<std::string> names;
    std::vector<std::string> families;
    for (option::Option* opt = pairs_opt; opt; opt = opt->next())
    {
      std::string pair_string = std::string(opt->arg);
      auto split_point = pair_string.find('|');
      families.push_back(pair_string.substr(0, split_point));
      names.push_back(pair_string.substr(split_point + 1));
    }
    group = lookup.getGroupFromNames(names, families, timeout_ms);
  }
  else if (macs_opt)
  {
    std::vector<hebi::MacAddress> macs;
    for (option::Option* opt = macs_opt; opt; opt = opt->next())
    {
      hebi::MacAddress mac;
      mac.setToHexString(std::string(opt->arg));
      macs.push_back(mac);
    }
    group = lookup.getGroupFromMacs(macs, timeout_ms);
  }
  else // The right combination of names/families has been set...there is more
       // complicated validation logic for this case, so we leave it as the
       // 'else' case.
  {
    std::vector<std::string> names;
    std::vector<std::string> families;
    for (option::Option* opt = names_opt; opt; opt = opt->next())
      names.push_back(std::string(opt->arg));
    for (option::Option* opt = families_opt; opt; opt = opt->next())
      families.push_back(std::string(opt->arg));
    group = lookup.getGroupFromNames(names, families, timeout_ms);
  }

  // Print out debug info if requested
  if (printModuleList)
  {
    std::cout << "Modules found on network (Family|Name):" << std::endl;
    std::shared_ptr<hebi::Lookup::EntryList> entry_list = lookup.getEntryList();
    for (auto entry : *entry_list)
    {
      std::cout << entry.family_ << " | " << entry.name_ << std::endl;
    }
    std::cout << std::endl;
  }

  return group;
}
