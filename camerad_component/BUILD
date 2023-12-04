load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "libcamerad_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":camerad_component_lib",
    ]
)

cc_library(
    name = "camerad_component_lib",
    srcs = ["camerad_component.cc"],
    hdrs = ["camerad_component.h"],
    alwayslink = True,
    copts = ["-std=c++17"],
    deps = [
        "//cyber",
        "//perception/camerad_component/cameras:camera_common",
        "//perception/camerad_component/proto:camera_conf_cc_proto",
        "//common_msgs/camerad:frame_data_cc_proto",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:videoio",
        "@xnxpilot//:libyuv",
    ]
)