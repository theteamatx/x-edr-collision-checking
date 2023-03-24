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
    cc_api_version = 2,
    deps = [
        "//experimental/users/buschmann/collision_checking/geometry_shapes:eigen_proto",
        "//experimental/users/buschmann/collision_checking/geometry_shapes:proto",
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
        "//experimental/users/buschmann/collision_checking/geometry_shapes:eigen_proto_utils",
        "//experimental/users/buschmann/collision_checking/geometry_shapes:utils",
        "//third_party/absl/status",
        "//third_party/absl/status:statusor",
        "//third_party/absl/strings",
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
        "//experimental/users/buschmann/collision_checking:eigenmath",
        "//experimental/users/buschmann/collision_checking/geometry_shapes",
        "//experimental/users/buschmann/collision_checking/geometry_shapes:geometry_shape",
        "//googlex/proxy/iterators",
        "//third_party/absl/status",
        "//third_party/absl/status:statusor",
        "//third_party/absl/strings",
        "//third_party/absl/strings:str_format",
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
        "//experimental/users/buschmann/collision_checking:test_utils",
        "//experimental/users/buschmann/collision_checking/geometry_shapes",
        "//third_party/absl/status",
        "//third_party/googletest:gtest_main_no_google3",
        "//third_party/googletest:gtest_no_google3",
    ],
)

cc_library(
    name = "assembly_connectivity",
    srcs = ["assembly_connectivity.cc"],
    hdrs = ["assembly_connectivity.h"],
    deps = [
        ":assembly",
        "//third_party/absl/container:flat_hash_map",
        "//third_party/absl/container:flat_hash_set",
        "//third_party/absl/strings",
        "//third_party/absl/types:span",
    ],
)

cc_test(
    name = "assembly_connectivity_test",
    srcs = ["assembly_connectivity_test.cc"],
    deps = [
        ":assembly",
        ":assembly_connectivity",
        ":proto_utils",
        "//experimental/users/buschmann/collision_checking:test_utils",
        "//third_party/googletest:gtest_main_no_google3",
        "//third_party/googletest:gtest_no_google3",
    ],
)

cc_test(
    name = "proto_utils_test",
    srcs = ["proto_utils_test.cc"],
    deps = [
        ":cc_proto",
        ":proto_utils",
        "//experimental/users/buschmann/collision_checking:test_utils",
        "//third_party/googletest:gtest_main_no_google3",  # build_cleaner: keep
    ],
)
