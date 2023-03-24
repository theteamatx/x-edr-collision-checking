#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DEBUG_OPTIONS_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DEBUG_OPTIONS_H_

// Enumeration of options for enabling or disabling features useful for
// debugging.

namespace collision_checking {

// Options for which enabling and disabling features useful for debugging.
enum DebugOptions : unsigned {
  // Nothing set.
  kDebugOptionsNone = 0x0,
  // Perform expensive input parameter checks.
  kDebugOptionsPerformExpensiveInputChecks = 0x1
};
}  // namespace collision_checking
#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DEBUG_OPTIONS_H_
