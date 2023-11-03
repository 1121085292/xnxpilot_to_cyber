load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "opencl",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    linkopts = [
        "-L/home/xzy/DowmLoads/apollo-master/xnxpilot/opencl/lib",
        "-lOpenCL",
    ],
)