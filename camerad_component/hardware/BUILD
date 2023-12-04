load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hw",
    hdrs = [
        "hw.h",
        "base.h"
    ],
    copts = ["-std=c++17"],
    deps = [
        "//perception/camerad_component/hardware/jetson:hardware",
    ]
)