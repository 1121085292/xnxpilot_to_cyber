load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "visionipc_server",
    srcs = ["visionipc_server.cc"],
    hdrs = ["visionipc_server.h"],
    copts = ["-std=c++17"],
    deps = [
        "//cyber",
        ":ipc",
        ":visionbuf",
        "@zmq",
    ]
)

cc_library(
    name = "visionbuf",
    srcs = ["visionbuf.cc"],
    hdrs = ["visionbuf.h",
            "visionipc.h"],
    copts = ["-std=c++17"],
    deps = [
        "@opencl",
    ]
)

cc_library(
    name = "ipc",
    srcs = ["ipc.cc"],
    hdrs = ["ipc.h"],
    deps = [
        "//xnxpilot/phonelibs/json11:json11",
    ]
)

cpplint()