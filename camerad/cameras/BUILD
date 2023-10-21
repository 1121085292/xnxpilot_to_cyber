load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tool/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_mipi",
    srcs = ["camera_mipi.cc"],
    hdrs = ["camera_mipi.h"],
    deps = [
        ":camera_common",
        "@opencl",
        "@opencv",
        "@opencv//:core",
        "@opencv//:highgui",
    ]
)

cc_library(
    name = "camera_common",
    srcs = ["camera_common.cc"],
    hdrs = ["camera_common.h"],
    deps = [
        "//cyber",
        "//selfdrive/common:common",
        "//selfdrive/camerad/transforms:rgb_to_yuv",
        "//cereal/visionipc:visionipc_server",     
    ]
)