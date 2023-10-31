load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "gles3",
    srcs = glob(["lib/*.so"]),
    hdrs = glob(["include/**/*.h"]),
    strip_include_prefix = "include",
)

install(
    name = "install",
    data_dest = "gles3",
    data = [
        ":cyberfile.xml",
        ":gles3.BUILD",
    ],
)

install_src_files(
    name = "install_src_lib",
    src_dir = ["lib"],
    dest = "gles3/lib",
    filter = "lib*.so*",
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "gles3",
    filter = "*",
    deps = ["install_src_lib"]
)