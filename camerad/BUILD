load("//tools/install:install.bzl", "install", "install_src_files")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "libcamerad_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":camerad_component_lib",
    ]
)

cc_library(
    name = "camerad_component_lib",
    srcs = ["camerad_component.cc"],
    hdrs = ["camerad_component.h"],
    deps = [
        "//cyber",
        "//xnxpilot/selfdrive/camerad/cameras:camera_common",
        "//xnxpilot/selfdrive/camerad/proto:camera_conf_cc_proto",
        "//xnxpilot/common_msgs/camerad:frame_data_cc_proto",
        "@opencv//:core",
        "@opencv//:highgui",
    ]

)

filegroup(
    name = "conf",
    srcs = [
        ":camerad.dag",
        ":camerad.launch",
    ],
)

install(
    name = "install",
    data = [
        "//xnxpilot/common_msgs/camerad:frame_data_cc_proto",
        "//xnxpilot/selfdrive/camerad/conf:camera_conf_cc_proto",
        "cyberfile.xml",
    ],
    data_dest = "camerad",
    library_dest = "camerad/lib",
    library_strip_prefix = ["component", "proto"],
    targets = [
        ":libcamerad_component.so",
    ],
    deps = [
        "pb_headers",
        ":dag",
        ":launch",
    ],
)

install(
    name = "pb_headers",
    data = [
        "//xnxpilot/common_msgs/camerad:frame_data_cc_proto",
        "//xnxpilot/selfdrive/camerad/proto:camera_conf_cc_proto",
    ],
    data_dest = "camerad/include"
)

install(
    name = "dag",
    data = [":camerad.dag"],
    data_dest = "camerad/dag"
)

install(
    name = "launch",
    data = [":camerad.launch"],
    data_dest = "camerad/launch"
)

install_src_files(
    name = "headers",
    src_dir = ["."],
    dest = "camerad/include",
    filter = "*.h",
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "camerad/component",
    filter = "*",
    deps = [
        ":headers"
    ],
)

cpplint()