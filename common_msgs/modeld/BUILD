load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

#生成 proto 库
proto_library(
    name = "camera_odometry_proto",
    srcs = ["camera_odometry.proto",],
    deps = [
        "//modules/common_msgs/basic_msgs:error_code_proto"
    ]
)

#生成 C++ 相关的源文件
cc_proto_library(
    name = "camera_odometry_cc_proto",
    deps = [":camera_odometry_proto",],
)

#生成 python 相关的源文件
py_proto_library(
   name = "camera_odometry_py_proto",
   deps = [":camera_odometry_proto"],
)

#==============================================
#生成 proto 库
proto_library(
    name = "model_data_v2_proto",
    srcs = ["model_data_v2.proto",],
    deps = [
        "//modules/common_msgs/basic_msgs:error_code_proto"
    ]
)
#生成 C++ 相关的源文件
cc_proto_library(
    name = "model_data_v2_cc_proto",
    deps = [":model_data_v2_proto",],
)

#生成 python 相关的源文件
py_proto_library(
   name = "model_data_v2_py_proto",
   deps = [":model_data_v2_proto"],
)
