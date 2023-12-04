load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "impl_msgq",
    srcs = ["impl_msgq.cc"],
    hdrs = [
        "impl_msgq.h",
        "services.h",
        "messaging.h"
    ],
    copts = ["-std=c++17"],
    deps = [
        ":msgq",
        "@xnxpilot//:zmq",
    ]
)

cc_library(
    name = "impl_zmq",
    srcs = ["impl_zmq.cc"],
    hdrs = [
        "impl_zmq.h",
        "services.h",
        "messaging.h"
    ],
    copts = ["-std=c++17"],
    deps = [
        "@xnxpilot//:zmq",
    ]
)

cc_library(
    name = "messaging",
    srcs = ["messaging.cc"],
    hdrs = [
        "messaging.h",
    ],
    copts = ["-std=c++17"],
    deps = [
        ":impl_zmq",
        ":impl_msgq",
    ]
)

cc_library(
    name = "msgq",
    srcs = ["msgq.cc"],
    hdrs = [
        "msgq.h",
    ],
    copts = ["-std=c++17"],
)