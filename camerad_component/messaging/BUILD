load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "impl_msgq",
    srcs = ["impl_msgq.cpp"],
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
    srcs = ["impl_zmq.cpp"],
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
    srcs = ["messaging.cpp"],
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
    srcs = ["msgq.cpp"],
    hdrs = [
        "msgq.h",
    ],
    copts = ["-std=c++17"],
)