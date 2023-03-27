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
    name = "eigen_proto",
    srcs = [
        "eigen.proto",
    ],
)

cc_proto_library(
    name = "eigen_cc_proto",
    deps = [":eigen_proto"],
)

cc_library(
    name = "eigen_proto_utils",
    hdrs = [
        "eigen_proto_utils.h",
    ],
    deps = [
        ":eigen_cc_proto",
        "//collision_checking:eigenmath",
        "//collision_checking:logging",
    ],
)

proto_library(
    name = "proto",
    srcs = [
        "geometry_shapes.proto",
    ],
    deps = [":eigen_proto"],
)

cc_proto_library(
    name = "cc_proto",
    deps = [":proto"],
)

cc_library(
    name = "utils",
    srcs = [
        "proto_utils.cc",
        "utils_impl.h",
    ],
    hdrs = [
        "proto_utils.h",
        "utils_impl.h",
    ],
    deps = [
        ":cc_proto",
        ":eigen_proto_utils",
        ":geometry_shape",
        ":geometry_shapes",
        "//collision_checking:eigenmath",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "geometry_shape",
    hdrs = [
        "shape_base.h",
    ],
    deps = [
        "//collision_checking:eigenmath",
        "//collision_checking:logging",
    ],
)

cc_library(
    name = "geometry_shapes",
    hdrs = [
        "box.h",
        "capsule.h",
        "composite_shape.h",
        "sphere.h",
        "spheres.h",
    ],
    deps = [
        ":geometry_shape",
        "//collision_checking:eigenmath",
    ],
)
