load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    hdrs = [
        "mat.h",
        "modeldata.h",
    ],
    copts = ["-std=c++17"],
    deps = [
        ":clutil",
        ":swaglog",
        ":visionimg",
        ":queue",
        "//xnxpilot/selfdrive/hardware:hw",
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
    copts = ["-std=c++17"],
)

cc_library(
    name = "queue",
    hdrs = ["queue.h"],
    copts = ["-std=c++17"],
)

cc_library(
    name = "params",
    srcs = ["params.cc"],
    copts = ["-std=c++17"],
    deps = [
        ":swaglog",
        "//xnxpilot/selfdrive/hardware:hw",
    ]
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
        "//xnxpilot/phonelibs//json11:json11",
        "//xnxpilot/selfdrive/hardware:hw",
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
        "//xnxpilot/cereal/visionipc:visionbuf",
        "@gles3",
    ]
)

exports_files([
    "params.h",
])
