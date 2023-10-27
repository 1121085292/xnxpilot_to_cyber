load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hw",
    hdrs = [
        "hw.h",
        "base.h",
        "//xnxpilot/selfdrive/hardware/jetson:hardware.h",
        "//xnxpilot/selfdrive/common:params.h",
    ],
    copts = ["-std=c++"],
)