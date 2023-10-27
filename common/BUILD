load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    hdrs = [
        "mat.h",
        "queue.h",
    ]
    deps = [
        "clutil",
        "swaglog",
        "visionimg",
    ]
)

cc_library(
    name = "clutil",
    srcs = ["clutil.cc"],
    hdrs = ["clutil.h"],
    deps = [
        "util",
        "@opencl",
    ]
)

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
)

cc_library(
    name = "swaglog"
    srcs = ["swaglog.cc"],
    hdrs = [
        "swaglog.h",
        "timing.h",
        "version.h",
    ],
    deps = [
        "util",
        "//phonelibs//json11:json11",
        "//selfdrive/hardware:hw",
        "@zmq",
    ]
)

cc_library(
    name = "visionimg"
    srcs = ["visionimg.cc"],
    hdrs = [
        "visionimg.h"
    ],
    deps = [
        "//cereal/visionipc:visionbuf",
        "@Gles",
    ]
)
