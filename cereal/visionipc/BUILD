load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tool/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        "cyberfile.xml",
    ],
    data_dest = "visionipc",
    library_dest = "visionipc/lib",
    library_strip_prefix = ["src"],
    targets = [
        "//xnxpilot/cereal/visionipc:visionipc_server",
    ],
    deps = [
        "pb_headers",
    ],
)

install(
    name = "pb_headers",
    data_dest = "visionipc/include"
)

install_src_files(
    name = "headers",
    src_dir = ["."],
    dest = "visionipc/include",
    filter = "*.h",
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "visionipc/component",
    filter = "*",
    deps = [
        ":headers"
    ],
)

cc_library(
    name = "visionipc_server",
    srcs = ["visionipc_server.cc"],
    hdrs = ["visionipc_server.h"],
    copts = ["-std=c++17"],
    deps = [
        "//cyber",
        ":visionbuf",
        ":ipc",
        "//xnxpilot/third_party/zmq"
    ]
)

cc_library(
    name = "visionbuf",
    srcs = ["visionbuf.cc"],
    hdrs = ["visionbuf.h",
            "visionipc.h"],
    copts = ["-std=c++17"],
    deps = [
        "//xnxpilot/third_party/opencl",
    ]
)

cc_library(
    name = "ipc",
    srcs = ["ipc.cc"],
    hdrs = ["ipc.h"],
)