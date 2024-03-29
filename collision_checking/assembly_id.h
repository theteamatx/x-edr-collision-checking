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

#ifndef COLLISION_CHECKING_ASSEMBLY_ID_H_
#define COLLISION_CHECKING_ASSEMBLY_ID_H_

#include <cstdint>

#include "collision_checking/typed_id_int.h"

namespace collision_checking {

// Id type for assemblies.
using AssemblyId = TypedIdInt<uint8_t>;

}  // namespace collision_checking

#endif  // COLLISION_CHECKING_ASSEMBLY_ID_H_
