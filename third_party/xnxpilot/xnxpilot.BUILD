load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "opencl",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-L/home/xzy/DowmLoads/apollo-master/xnxpilot/lib",
        "-lOpenCL",
    ],
)

cc_library(
    name = "zmq",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-L/home/xzy/DowmLoads/apollo-master/xnxpilot/lib",
        "-lzmq",
    ],
)

cc_library(
    name = "gles3",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-L/home/xzy/DowmLoads/apollo-master/xnxpilot/lib",
        "-lGLESv2",
    ],
)

cc_library(
    name = "jpeglib",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-L/home/xzy/DowmLoads/apollo-master/xnxpilot/lib",
        "-ljpeg",
    ],
)

cc_library(
    name = "boost",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-L/home/xzy/DowmLoads/apollo-master/xnxpilot/lib",
        "-lboost_filesystem",
        "-lboost_data_time",
        "-lboost_iostream",
        "-lboost_locale",
        "-lboost_thread",
    ],
)