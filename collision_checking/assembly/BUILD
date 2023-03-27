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

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_proto_library")

licenses(["notice"])

exports_files(["LICENSE"])

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "proto",
    srcs = [
        "assembly.proto",
    ],
    deps = [
        "//collision_checking/geometry_shapes:eigen_proto",
        "//collision_checking/geometry_shapes:proto",
    ],
)

cc_proto_library(
    name = "cc_proto",
    deps = [":proto"],
)

cc_library(
    name = "proto_utils",
    srcs = [
        "proto_utils.cc",
        "utils_impl.h",
    ],
    hdrs = [
        "proto_utils.h",
    ],
    deps = [
        ":assembly",
        ":cc_proto",
        "//collision_checking/geometry_shapes:eigen_proto_utils",
        "//collision_checking/geometry_shapes:utils",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "assembly",
    srcs = [
        "assembly.cc",
        "geometry.cc",
        "joint.cc",
        "link.cc",
    ],
    hdrs = [
        "assembly.h",
        "geometry.h",
        "joint.h",
        "link.h",
    ],
    deps = [
        "//collision_checking:eigenmath",
        "//collision_checking/geometry_shapes",
        "//collision_checking/geometry_shapes:geometry_shape",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@genit//genit:iterators",
    ],
)

cc_test(
    name = "assembly_test",
    srcs = [
        "assembly_test.cc",
    ],
    deps = [
        ":assembly",
        ":cc_proto",
        ":proto_utils",
        "//collision_checking:test_utils",
        "//collision_checking/geometry_shapes",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest_main",
        "@eigenmath//eigenmath:matchers",
    ],
)

cc_library(
    name = "assembly_connectivity",
    srcs = ["assembly_connectivity.cc"],
    hdrs = ["assembly_connectivity.h"],
    deps = [
        ":assembly",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "assembly_connectivity_test",
    srcs = ["assembly_connectivity_test.cc"],
    deps = [
        ":assembly",
        ":assembly_connectivity",
        ":proto_utils",
        "//collision_checking:test_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "proto_utils_test",
    srcs = ["proto_utils_test.cc"],
    deps = [
        ":cc_proto",
        ":proto_utils",
        "//collision_checking:test_utils",
        "@com_google_googletest//:gtest_main",
        "@eigenmath//eigenmath:matchers",
    ],
)
