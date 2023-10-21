load("//tools/install:install.bzl", "install", "install_src_files")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "libcamerad_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        "//selfdrive/camerad/camerad:camera_mipi",
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
        "//common_msgs/camerad:frame_data_cc_proto",
        "//selfdrive/camerad/conf:camera_conf_cc_proto",
        "cyberfile.xml",
    ],
    data_dest = "camerad_component",
    library_dest = "camerad_component/lib",
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
        "//common_msgs/camerad:frame_data_cc_proto",
        "//selfdrive/camerad/conf:camera_conf_cc_proto",
    ],
    data_dest = "camerad_component/include"
)

install(
    name = "dag",
    data = [":cipv.dag"],
    data_dest = "camerad_component/dag"
)

install(
    name = "launch",
    data = [":cipv.launch"],
    data_dest = "camerad_component/launch"
)

install_src_files(
    name = "headers",
    src_dir = ["."],
    dest = "camerad_component/include",
    filter = "*.h",
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "camerad_component/component",
    filter = "*",
    deps = [
        ":headers"
    ],
)
