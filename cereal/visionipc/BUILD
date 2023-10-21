load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tool/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "visionipc_server",
    srcs = ["visionipc_server.cc"],
    hdrs = ["visionipc_server.h"],
    deps = [
        "//cyber",
        ":visionipc",
        ":visionbuf",
        ":ipc",
    ]
)

cc_library(
    name = "visionbuf",
    srcs = ["visionbuf.cc"],
    hdrs = ["visionbuf.h"],
    deps = [
        "visionipc",
        "@opencl"
    ]
)


cc_library(
    name = "visionipc",
    hdrs = ["visionipc.h"],
)

cc_library(
    name = "ipc",
    srcs = ["ipc.cc"],
    hdrs = ["ipc.h"],
)