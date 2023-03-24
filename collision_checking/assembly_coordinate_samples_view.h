#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COORDINATE_SAMPLES_VIEW_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COORDINATE_SAMPLES_VIEW_H_

#include "collision_checking/assembly_kinematics.h"
#include "third_party/absl/types/span.h"

namespace collision_checking {

// A view of a number of coordinate samples.
template <typename Scalar>
class AssemblyCoordinateSamplesView {
 public:
  using NonConstScalar = std::remove_const_t<Scalar>;
  using ConstScalar = std::add_const_t<Scalar>;

  AssemblyCoordinateSamplesView() = delete;
  AssemblyCoordinateSamplesView(const AssemblyCoordinateSamplesView&) = default;

  // Creates an AssemblyCoordinateSamplesView.
  AssemblyCoordinateSamplesView(absl::Span<Scalar> span, int joint_count,
                                int sample_count)
      : span_(span), joint_count_(joint_count), sample_count_(sample_count) {}

  // Returns the number of joints (non base-pose degrees of freedom).
  std::size_t JointCount() const { return joint_count_; }
  // Returns the number of samples.
  std::size_t SampleCount() const { return sample_count_; }
  // Returns a const Span of all coordinate samples, which is the concatenation
  // of AssemblyCoordintes in the order of samples.
  absl::Span<ConstScalar> ConstSpan() const { return span_; }
  // Returns a const view on the coordintes for `sample`.
  AssemblyCoordinateView<ConstScalar> ConstSample(int sample) const {
    return AssemblyCoordinateView<ConstScalar>(
        span_.data() +
            sample * (joint_count_ +
                      AssemblyCoordinateView<ConstScalar>::kScalarsPerPose),
        joint_count_);
  }
  // Returns a possibly non-const view on the coordinates at `samples`.
  AssemblyCoordinateView<Scalar> Sample(int sample) {
    return AssemblyCoordinateView<Scalar>(
        span_.data() +
            sample * (joint_count_ +
                      AssemblyCoordinateView<Scalar>::kScalarsPerPose),
        joint_count_);
  }

 private:
  absl::Span<Scalar> span_;
  std::size_t joint_count_ = 0;
  std::size_t sample_count_ = 0;
};
}  // namespace collision_checking
#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COORDINATE_SAMPLES_VIEW_H_
