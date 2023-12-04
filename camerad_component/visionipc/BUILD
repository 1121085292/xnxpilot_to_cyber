load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "visionipc_client",
    srcs = ["visionipc_client.cc"],
    hdrs = ["visionipc_client.h"],
    copts = ["-std=c++17"],
    deps = [
        ":visionipc_server"
    ]
)

cc_library(
    name = "visionipc_server",
    srcs = ["visionipc_server.cc"],
    hdrs = ["visionipc_server.h"],
    copts = ["-std=c++17"],
    deps = [
        "//cyber",
        ":ipc",
        ":visionbuf",
        "//perception/camerad_component/messaging:messaging",
        "@xnxpilot//:zmq",
    ]
)

cc_library(
    name = "visionbuf",
    srcs = ["visionbuf.cc",
            "visionbuf_cl.cc"
    ],
    hdrs = ["visionbuf.h",
            "visionipc.h"],
    copts = ["-std=c++17"],
    deps = [
        "@opencl//:opencl",
    ]
)

cc_library(
    name = "ipc",
    srcs = ["ipc.cc"],
    hdrs = ["ipc.h"],
    copts = ["-std=c++17"],
    deps = [
        "//perception/camerad_component/json11:json11",
    ]
)

cpplint()