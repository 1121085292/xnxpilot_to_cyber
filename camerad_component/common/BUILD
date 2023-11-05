load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

# cc_library(
#     name = "common",
#     hdrs = glob(["*.h"]),
#     copts = ["-std=c++17"],
#     deps = [
#         ":clutil",
#         ":swaglog",
#         ":visionimg",
#         ":queue",
#         "//xnxpilot/selfdrive/hardware:hw",
#     ]
# )

cc_library(
    name = "clutil",
    srcs = ["clutil.cc"],
    hdrs = ["clutil.h"],
    deps = [
        ":util",
        "@xnxpilot//:opencl",
    ],
    copts = ["-std=c++17"],
)

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    copts = ["-std=c++17"],
)

# cc_test(
#     name = "clutil_test",
#     srcs = ["clutil_test.cc"],
#     deps = [
#         ":clutil",
#         "@opencl",
#         "@com_google_googletest//:gtest_main",
#     ]
# )

# cc_library(
#     name = "queue",
#     hdrs = ["queue.h"],
#     copts = ["-std=c++17"],
# )

# cc_library(
#     name = "params",
#     srcs = ["params.cc"],
#     copts = ["-std=c++17"],
#     deps = [
#         ":swaglog",
#         "//xnxpilot/selfdrive/hardware:hw",
#     ]
# )

# cc_library(
#     name = "swaglog"
#     srcs = ["swaglog.cc"],
#     hdrs = [
#         "swaglog.h",
#         "timing.h",
#         "version.h",
#     ],
#     deps = [
#         "util",
#         "//xnxpilot/phonelibs//json11:json11",
#         "//xnxpilot/selfdrive/hardware:hw",
#         "@zmq",
#     ]
# )

# cc_library(
#     name = "visionimg"
#     srcs = ["visionimg.cc"],
#     hdrs = [
#         "visionimg.h"
#     ],
#     deps = [
#         "//xnxpilot/cereal/visionipc:visionbuf",
#         "@gles3",
#     ]
# )

# exports_files([
#     "params.h",
# ])
