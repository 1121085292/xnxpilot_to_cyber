load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hw",
    hdrs = [
        "hw.h",
        "jetson/base.h",
    ],
    deps = [
        "//selfdrive/common:util",
        "//selfdrive/common:params",
    ]
)