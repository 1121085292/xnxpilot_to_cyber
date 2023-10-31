load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "opencl",
    srcs = glob(["lib/*.so"]),
    hdrs = glob(["include/**/*.h"]),
    strip_include_prefix = "include",
)

install(
    name = "install",
    data_dest = "opencl",
    data = [
        ":cyberfile.xml",
        ":opencl.BUILD",
    ],
)

install_src_files(
    name = "install_src_lib",
    src_dir = ["lib"],
    dest = "opencl/lib",
    filter = "lib*.so*",
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "opencl",
    filter = "*",
    deps = ["install_src_lib"]
)