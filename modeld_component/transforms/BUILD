load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "loadyuv",
    srcs = ["loadyuv.cc"],
    hdrs = ["loadyuv.h"],
    copts = ["-std=c++17"],
    deps = [
        "//perception/camerad_component/common:clutil",
    ]
)

cc_library(
    name = "transform",
    srcs = ["transform.cc"],
    hdrs = ["transform.h"],
    copts = ["-std=c++17"],
    deps = [
        "//perception/camerad_component/common:common",
    ]
)