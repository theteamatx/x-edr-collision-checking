#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_EIGENMATH_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_EIGENMATH_H_
#include "collision_checking/inlining.h"
#include "googlex/proxy/eigenmath/line_search.h"
#include "googlex/proxy/eigenmath/pose3.h"
#include "googlex/proxy/eigenmath/types.h"
#include "googlex/proxy/eigenmath/vector_utils.h"

namespace collision_checking {

constexpr inline auto kEigenDefaultOptions = ::blue::eigenmath::kDefaultOptions;
constexpr inline auto kMaxEigenVectorCapacity =
    ::blue::eigenmath::kMaxEigenVectorCapacity;

// Vector/Matrix types used in these libraries.
template <typename Scalar>
using Vector2 = ::blue::eigenmath::Vector2<Scalar>;
template <typename Scalar>
using Vector3 = ::blue::eigenmath::Vector3<Scalar>;
template <typename Scalar>
using Vector4 = ::blue::eigenmath::Vector4<Scalar>;
template <typename Scalar>
using VectorN = ::blue::eigenmath::VectorN<Scalar>;
template <typename Scalar>
using VectorX = ::blue::eigenmath::VectorX<Scalar>;
template <typename Scalar>
using Matrix3 = ::blue::eigenmath::Matrix3<Scalar>;
template <typename Scalar>
using Pose3 = ::blue::eigenmath::Pose3<Scalar>;
template <typename Scalar>
using Quaternion = ::blue::eigenmath::Quaternion<Scalar>;
template <typename Scalar>
using SO3 = ::blue::eigenmath::SO3<Scalar>;

using Vector2d = Vector2<double>;
using Vector3d = Vector3<double>;
using Vector4d = Vector4<double>;
using VectorNd = VectorN<double>;
using VectorXd = VectorX<double>;
using Pose3d = Pose3<double>;
using Quaterniond = Quaternion<double>;
using SO3d = SO3<double>;
using Matrix3d = Matrix3<double>;

//
template <typename Functor, typename Scalar>
CC_INLINE std::pair<Scalar, Scalar> GoldenSectionSearchMinimize(
    Scalar left, Scalar right, Functor f, Scalar x_tolerance) {
  return ::blue::eigenmath::GoldenSectionSearchMinimize(left, right, f,
                                                        x_tolerance);
}

template <typename T>
inline constexpr T Saturate(const T& val, const T& min_max) {
  return std::clamp(val, -min_max, min_max);
}

template <class Scalar>
inline std::array<Vector3<Scalar>, 2> ExtendToOrthonormalBasis(
    const Vector3<Scalar>& u){
  return ::blue::eigenmath::ExtendToOrthonormalBasis(u);
    }

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_EIGENMATH_H_
