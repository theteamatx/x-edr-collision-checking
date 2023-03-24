#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_EIGEN_PROTO_UTILS_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_EIGEN_PROTO_UTILS_H_

#include "experimental/users/buschmann/collision_checking/eigenmath.h"
#include "experimental/users/buschmann/collision_checking/geometry_shapes/eigen.proto.h"
#include "experimental/users/buschmann/collision_checking/logging.h"

namespace collision_checking {

inline void ProtoFromPose(const Pose3d& pose, proto::Pose3dProto* proto) {
  CC_CHECK_NE(proto, nullptr);
  proto->set_tx(pose.translation().x());
  proto->set_ty(pose.translation().y());
  proto->set_tz(pose.translation().z());
  proto->set_rx(pose.quaternion().x());
  proto->set_ry(pose.quaternion().y());
  proto->set_rz(pose.quaternion().z());
  proto->set_rw(pose.quaternion().w());
}

inline void ProtoFromVector(const Vector3d& vector,
                            proto::Vector3dProto* proto) {
  CC_CHECK_NE(proto, nullptr);
  proto->add_vec(vector.x());
  proto->add_vec(vector.y());
  proto->add_vec(vector.z());
}

inline proto::Matrix3dProto ProtoFromMatrix3d(const Matrix3d& matrix) {
  proto::Matrix3dProto proto;
  proto.add_mat(matrix(0, 0));
  proto.add_mat(matrix(0, 1));
  proto.add_mat(matrix(0, 2));
  proto.add_mat(matrix(1, 0));
  proto.add_mat(matrix(1, 1));
  proto.add_mat(matrix(1, 2));
  proto.add_mat(matrix(2, 0));
  proto.add_mat(matrix(2, 1));
  proto.add_mat(matrix(2, 2));
  return proto;
}

inline Matrix3d EigenMatrixFromProto(const proto::Matrix3dProto& proto) {
  if (proto.mat_size() == 9) {
    Matrix3d matrix;
    matrix(0, 0) = proto.mat(0);
    matrix(0, 1) = proto.mat(1);
    matrix(0, 2) = proto.mat(2);
    matrix(1, 0) = proto.mat(3);
    matrix(1, 1) = proto.mat(4);
    matrix(1, 2) = proto.mat(5);
    matrix(2, 0) = proto.mat(6);
    matrix(2, 1) = proto.mat(7);
    matrix(2, 2) = proto.mat(8);
    return matrix;
  } else {
    return Matrix3d::Zero();
  }
}

inline Vector3d EigenVectorFromProto(const proto::Vector3dProto& proto) {
  if (proto.vec_size() == 3) {
    return Vector3d(proto.vec(0), proto.vec(1), proto.vec(2));
  } else {
    return Vector3d::Zero();
  }
}

inline proto::Vector3dProto ProtoFromEigenVector(const Vector3d& vec) {
  proto::Vector3dProto proto;
  proto.add_vec(vec.x());
  proto.add_vec(vec.y());
  proto.add_vec(vec.z());
  return proto;
}

inline Pose3d PoseFromProto(const proto::Pose3dProto& proto) {
  // If the rotational component is empty, use a unit quaternion.
  const Quaterniond quaternion =
      proto.rx() != 0.0 || proto.ry() != 0.0 || proto.rz() != 0.0 ||
              proto.rw() != 0.0
          ? Quaterniond(proto.rw(), proto.rx(), proto.ry(), proto.rz())
          : Quaterniond::Identity();
  return Pose3d(quaternion, Vector3d(proto.tx(), proto.ty(), proto.tz()));
}
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_EIGEN_PROTO_UTILS_H_
