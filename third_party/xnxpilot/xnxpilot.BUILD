load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "opencl",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-Lxnxpilot/lib",
        "-lOpenCL",
    ],
)

cc_library(
    name = "zmq",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-Lxnxpilot/lib",
        "-lzmq",
    ],
)

cc_library(
    name = "gles3",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-Lxnxpilot/lib",
        "-lGLESv2",
    ],
)

cc_library(
    name = "libyuv",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-L/home/xzy/DowmLoads/apollo-master/xnxpilot/lib",
        "-lyuv",
    ],
)

cc_library(
    name = "jpeglib",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-Lxnxpilot/lib",
        "-ljpeg",
    ],
)
