load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera",
    srcs = ["camera.cc"],
    hdrs = ["camera.h"],
    deps = [
        ":orientation",
    ]
)

cc_library(
    name = "orientation",
    srcs = ["orientation.cc"],
    hdrs = ["orientation.hpp"],
    deps = [
        ":coordinates",
    ],
    copts = ["-std=c++17"],
)

cc_library(
    name = "coordinates",
    srcs = ["coordinates.cc"],
    hdrs = ["coordinates.hpp"],
    copts = ["-std=c++17"],
)