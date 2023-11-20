load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "onnx_model",
    srcs = ["onnxmodel.cc"],
    hdrs = ["onnxmodel.h",
            "runmodel.h"],
    copts = ["-std=c++17"],
    deps = [
        "//camerad_component/common:swaglog",
        "//camerad_component/common:util",
    ]
)