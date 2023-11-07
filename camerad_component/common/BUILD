load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    hdrs = glob(["*.h"]),
    copts = ["-std=c++17"],
    deps = [
        ":clutil",
        ":swaglog",
        ":visionimg",
        ":queue",
    ]
)

cc_library(
    name = "clutil",
    srcs = ["clutil.cc"],
    hdrs = ["clutil.h"],
    deps = [
        ":util",
        "@xnxpilot//:opencl",
    ],
    copts = ["-std=c++17"],
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
        "/camerad_component/hardware:hw",
    ]
)

cc_library(
    name = "swaglog",
    srcs = ["swaglog.cc"],
    hdrs = [
        "swaglog.h",
        "timing.h",
        "version.h",
    ],
    copts = ["-std=c++17"],
    deps = [
        ":util",
        "//camerad_component/json11:json11",
        "//camerad_component/hardware:hw",
        "@xnxpilot//:zmq",
    ]
)

cc_library(
    name = "visionimg"
    srcs = ["visionimg.cc"],
    hdrs = [
        "visionimg.h"
    ],
    copts = ["-std=c++17"],
    deps = [
        "//camerad_component/visionipc:visionbuf",
        "@xnxpilot//:gles3",
    ]
)