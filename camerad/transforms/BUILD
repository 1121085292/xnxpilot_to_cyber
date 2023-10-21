load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tool/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rgb_to_yuv",
    srcs = ["rgb_to_yuv.cc"],
    hdrs = ["rgb_to_yuv.h"],
    deps = [
        "//selfdrive/common:common",
    ]
)