# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

licenses(["notice"])

exports_files(["LICENSE"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vector",
    hdrs = ["vector.h"],
    deps = [
        ":inlining",
    ],
)

cc_test(
    name = "vector_test",
    srcs = ["vector_test.cc"],
    deps = [
        ":vector",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "assembly_id",
    hdrs = ["assembly_id.h"],
    deps = [":typed_id_int"],
)

cc_test(
    name = "id_assigner_test",
    srcs = ["id_assigner_test.cc"],
    deps = [
        ":id_assigner",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "composite_object",
    hdrs = ["composite_object.h"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":inlining",
        ":normalize_and_maybe_log",
        ":object_id",
        ":vector",
    ],
)

cc_test(
    name = "composite_object_test",
    srcs = ["composite_object_test.cc"],
    deps = [
        ":collision_checking_core",
        ":composite_object",
        ":eigenmath",
        ":test_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "collision_result",
    hdrs = ["collision_result.h"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":logging",
        ":vector",
    ],
)

cc_test(
    name = "collision_result_test",
    srcs = ["collision_result_test.cc"],
    deps = [
        ":collision_result",
        ":eigenmath",
        ":test_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "typed_id_int",
    hdrs = ["typed_id_int.h"],
)

cc_test(
    name = "typed_id_int_test",
    srcs = ["typed_id_int_test.cc"],
    deps = [
        ":typed_id_int",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "inlining",
    hdrs = ["inlining.h"],
)

cc_library(
    name = "logging",
    hdrs = ["logging.h"],
    deps = [
        ":inlining",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "normalize_and_maybe_log",
    hdrs = ["normalize_and_maybe_log.h"],
    deps = [
        ":eigenmath",
        ":inlining",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_library(
    name = "object_id",
    hdrs = ["object_id.h"],
    deps = [
        ":typed_id_int",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_test(
    name = "object_id_test",
    srcs = ["object_id_test.cc"],
    deps = [
        ":object_id",
        ":test_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "eigenmath",
    hdrs = [
        "eigenmath.h",
    ],
    deps = [
        ":inlining",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_library(
    name = "collision_checking_core",
    srcs = [
        "options.cc",
    ],
    hdrs = [
        "debug_options.h",
        "distance_box_box.h",
        "distance_point_box.h",
        "distance_point_point.h",
        "distance_point_segment.h",
        "distance_segment_box.h",
        "distance_segment_segment.h",
        "geometry.h",
        "options.h",
    ],
    deps = [
        ":eigenmath",
        ":inlining",
        ":logging",
        ":object_id",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_test(
    name = "options_test",
    srcs = ["options_test.cc"],
    deps = [
        ":collision_checking_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "geometry_test",
    srcs = ["geometry_test.cc"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":object_id",
        ":test_utils",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_library(
    name = "test_utils",
    testonly = True,
    srcs = ["test_utils.cc"],
    hdrs = ["test_utils.h"],
    deps = [
        ":logging",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
        "@eigen",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:matchers",
    ],
)

cc_test(
    name = "test_utils_test",
    srcs = ["test_utils_test.cc"],
    deps = [
        ":test_utils",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_benchmark//:benchmark",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "distance_point_point_test",
    srcs = ["distance_point_point_test.cc"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":test_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "distance_point_segment_test",
    srcs = ["distance_point_segment_test.cc"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":test_utils",
        "@com_google_absl//absl/flags:flag",
	"@com_google_absl//absl/log:absl_log",
        "@com_google_benchmark//:benchmark",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:sampling",
    ],
)

cc_test(
    name = "distance_segment_segment_test",
    srcs = ["distance_segment_segment_test.cc"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":test_utils",
        "@com_google_absl//absl/flags:flag",
        "@com_google_benchmark//:benchmark",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:sampling",
    ],
)

cc_test(
    name = "distance_point_box_test",
    srcs = ["distance_point_box_test.cc"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":test_utils",
        "@com_google_absl//absl/flags:flag",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:sampling",
    ],
)

cc_test(
    name = "distance_point_box_benchmark",
    srcs = ["distance_point_box_benchmark.cc"],
    deps = [
        ":collision_checking_core",
        "@com_google_benchmark//:benchmark",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_test(
    name = "distance_segment_box_test",
    srcs = ["distance_segment_box_test.cc"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":test_utils",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:sampling",
    ],
)

cc_binary(
    name = "distance_segment_box_benchmark",
    srcs = ["distance_segment_box_benchmark.cc"],
    deps = [
        ":collision_checking_core",
        "@com_google_benchmark//:benchmark",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_test(
    name = "distance_box_box_test",
    srcs = ["distance_box_box_test.cc"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":test_utils",
        "@com_google_absl//absl/flags:flag",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:sampling",
    ],
)

cc_binary(
    name = "distance_box_box_benchmark",
    srcs = ["distance_box_box_benchmark.cc"],
    deps = [
        ":collision_checking_core",
        "@com_google_benchmark//:benchmark",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_library(
    name = "id_assigner",
    hdrs = [
        "id_assigner.h",
    ],
    deps = [
        ":assembly_id",
        ":logging",
        ":object_id",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "voxel_code",
    hdrs = ["voxel_code.h"],
    deps = [":collision_checking_core"],
)

cc_test(
    name = "voxel_code_test",
    srcs = ["voxel_code_test.cc"],
    deps = [
        ":eigenmath",
        ":voxel_code",
        "@com_google_absl//absl/flags:flag",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "voxel_code_benchmark",
    srcs = ["voxel_code_benchmark.cc"],
    deps = [
        ":voxel_code",
        "@com_google_benchmark//:benchmark",
    ],
)

cc_library(
    name = "voxel_indexer",
    hdrs = ["voxel_indexer.h"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":logging",
        "@eigen",
    ],
)

cc_test(
    name = "voxel_indexer_test",
    srcs = ["voxel_indexer_test.cc"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":test_utils",
        ":voxel_indexer",
        "@com_google_absl//absl/flags:flag",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "voxel_map_object",
    hdrs = ["voxel_map_object.h"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":object_id",
        ":voxel_code",
        ":voxel_indexer",
        "@com_google_absl//absl/algorithm:container",
    ],
)

cc_test(
    name = "voxel_map_object_test",
    srcs = ["voxel_map_object_test.cc"],
    deps = [
        ":collision_checking_core",
        ":eigenmath",
        ":voxel_map_object",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "voxel_map_object_benchmark",
    srcs = ["voxel_map_object_benchmark.cc"],
    deps = [
        ":voxel_map_object",
        "@com_google_benchmark//:benchmark",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:sampling",
    ],
)

cc_library(
    name = "compute_collisions",
    hdrs = [
        "compute_collisions.h",
        "status.h",
    ],
    deps = [
        ":collision_checking_core",
        ":collision_result",
        ":composite_object",
        ":inlining",
        ":normalize_and_maybe_log",
        ":voxel_map_object",
        "@com_google_absl//absl/status",
    ],
)

cc_test(
    name = "compute_collisions_test",
    srcs = ["compute_collisions_test.cc"],
    deps = [
        ":collision_checking_core",
        ":compute_collisions",
        ":eigenmath",
        ":object_id",
        ":test_utils",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "assembly_coordinate_view",
    hdrs = ["assembly_coordinate_view.h"],
    deps = [
        ":inlining",
        "@eigen",
    ],
)

cc_library(
    name = "assembly_coordinates",
    hdrs = ["assembly_coordinates.h"],
    deps = [
        ":assembly_coordinate_view",
        ":eigenmath",
        ":inlining",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "assembly_kinematics",
    hdrs = [
        "assembly_kinematics.h",
    ],
    deps = [
        ":assembly_coordinate_view",
        ":eigenmath",
        ":inlining",
        ":object_id",
        ":vector",
        "//collision_checking/assembly",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "assembly_kinematics_test",
    srcs = ["assembly_kinematics_test.cc"],
    deps = [
        ":assembly_coordinates",
        ":assembly_kinematics",
        ":eigenmath",
        ":test_utils",
        "//collision_checking/assembly",
        "//collision_checking/assembly:cc_proto",
        "//collision_checking/assembly:proto_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "assembly_coordinate_samples_view",
    hdrs = ["assembly_coordinate_samples_view.h"],
    deps = [
        ":assembly_kinematics",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "model_interface",
    hdrs = ["model_interface.h"],
    deps = [
        ":assembly_id",
        ":compute_collisions",
        ":model_options",
        ":object_id",
        "//collision_checking/assembly",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "voxel_map_object_name",
    hdrs = [":voxel_map_object_name.h"],
    deps = ["@com_google_absl//absl/strings"],
)

proto_library(
    name = "bounding_box_proto",
    srcs = [
        "bounding_box.proto",
    ],
    deps = ["//collision_checking/geometry_shapes:eigen_proto"],
)

cc_proto_library(
    name = "bounding_box_cc_proto",
    deps = [":bounding_box_proto"],
)

proto_library(
    name = "collision_state_proto",
    srcs = ["collision_state.proto"],
    deps = [
        ":bounding_box_proto",
        "//collision_checking/geometry_shapes:proto",
    ],
)

cc_proto_library(
    name = "collision_state_cc_proto",
    deps = [":collision_state_proto"],
)

cc_library(
    name = "proto_utils",
    hdrs = [
        "proto_utils.h",
    ],
    deps = [
        ":bounding_box_cc_proto",
        ":collision_checking_core",
        ":collision_result",
        ":collision_state_cc_proto",
        ":composite_object",
        ":eigenmath",
        ":object_id",
        ":voxel_map_object",
        ":voxel_map_object_name",
        "//collision_checking/geometry_shapes:cc_proto",
        "//collision_checking/geometry_shapes:eigen_proto_utils",
        "@com_google_absl//absl/functional:function_ref",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "geometry_shape_conversion",
    srcs = [
        "geometry_shape_conversion.cc",
    ],
    hdrs = [
        "geometry_shape_conversion.h",
    ],
    deps = [
        ":collision_checking_core",
        ":composite_object",
        ":eigenmath",
        ":voxel_map_object",
        "//collision_checking/geometry_shapes",
        "//collision_checking/geometry_shapes:geometry_shape",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "geometry_shape_conversion_test",
    srcs = ["geometry_shape_conversion_test.cc"],
    deps = [
        ":collision_checking_core",
        ":composite_object",
        ":eigenmath",
        ":geometry_shape_conversion",
        ":test_utils",
        "//collision_checking/geometry_shapes",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "collision_checking_scratch",
    hdrs = [
        "collision_checking_scratch.h",
    ],
    deps = [
        ":assembly_kinematics",
        ":composite_object",
        ":inlining",
        ":vector",
    ],
)

cc_library(
    name = "assembly_collision_checker",
    hdrs = [
        "assembly_collision_checker.h",
    ],
    deps = [
        ":assembly_coordinates",
        ":assembly_id",
        ":assembly_kinematics",
        ":collision_checking_core",
        ":collision_checking_scratch",
        ":compute_collisions",
        ":eigenmath",
        ":geometry_shape_conversion",
        ":id_assigner",
        ":inlining",
        ":model_interface",
        ":model_options",
        ":object_id",
        ":proto_utils",
        ":vector",
        ":voxel_map_object_name",
        "//collision_checking/assembly",
        "//collision_checking/assembly:assembly_connectivity",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_test(
    name = "assembly_collision_checker_test",
    srcs = ["assembly_collision_checker_test.cc"],
    deps = [
        ":assembly_collision_checker",
        ":collision_checking_core",
        ":eigenmath",
        ":id_assigner",
        ":object_id",
        ":test_utils",
        ":vector",
        "//collision_checking/assembly:proto_utils",
        "//collision_checking/geometry_shapes",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "proto_utils_test",
    srcs = ["proto_utils_test.cc"],
    deps = [
        ":assembly_collision_checker",
        ":bounding_box_cc_proto",
        ":collision_checking_core",
        ":collision_state_cc_proto",
        ":eigenmath",
        ":object_id",
        ":proto_utils",
        ":test_utils",
        ":vector",
        "//collision_checking/assembly:cc_proto",
        "//collision_checking/assembly:proto_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "model_options",
    hdrs = [
        "model_options.h",
    ],
    deps = [
        ":inlining",
    ],
)
