// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
