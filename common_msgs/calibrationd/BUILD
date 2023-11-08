load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

#生成 proto 库
proto_library(
    name = "live_calibration_data_proto",
    srcs = ["live_calibration_data.proto",],
)

#生成 C++ 相关的源文件
cc_proto_library(
    name = "live_calibration_data_cc_proto",
    deps = [":live_calibration_data_proto",],
)

#生成 python 相关的源文件
py_proto_library(
   name = "live_calibration_data_py_proto",
   deps = [":live_calibration_data_proto"],
)
