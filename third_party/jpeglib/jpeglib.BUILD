load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "jpeglib",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-ljpeg",
    ],
    strip_include_prefix = "include",
)