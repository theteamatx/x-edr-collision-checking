#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_ID_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_ID_H_

#include <cstdint>

#include "experimental/users/buschmann/collision_checking/typed_id_int.h"

namespace collision_checking {

// Id type for assemblies.
using AssemblyId = TypedIdInt<uint8_t>;

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_ID_H_
