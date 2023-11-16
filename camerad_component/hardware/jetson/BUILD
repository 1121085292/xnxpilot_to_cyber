load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hardware",
    hdrs = [
        "hardware.h",
        "//camerad_component/common:params.h"
        "//camerad_component/common:util.h"
    ],
    copts = ["-std=c++17"],
)
