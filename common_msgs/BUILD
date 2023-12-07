load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "test",
    srcs = ["test.cc"],
    deps = [
      "//cyber",
      "//common_msgs/camerad:frame_data_cc_proto",
      "//common_msgs/plannerd:car_state_cc_proto",
      "//common_msgs/plannerd:lateral_plan_cc_proto",
    ],
)