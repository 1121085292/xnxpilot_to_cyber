load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "zmq",
    srcs = glob(["lib/*.so"]),
    hdrs = glob(["include/**/*.h"]),
    strip_include_prefix = "include",
)

install(
    name = "install",
    data_dest = "zmq",
    data = [
        ":cyberfile.xml",
        ":zmq.BUILD",
    ],
)

install_src_files(
    name = "install_src_lib",
    src_dir = ["lib"],
    dest = "zmq/lib",
    filter = "lib*.so*",
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "zmq",
    filter = "*",
    deps = ["install_src_lib"]
)