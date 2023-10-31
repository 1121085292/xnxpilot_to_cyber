load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gles3",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lGLESv2",
    ],
    strip_include_prefix = "include",
)