load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "kvaser_can",
    # linkopts = ["-L/usr/local/lib/"],
    hdrs = [
        "include/canlib.h",
        "include/canstat.h",
        "include/obsolete.h",
    ],
    srcs = [
        "lib/libcanlib.so",
        "lib/libcanlib.so.1",
        "lib/libcanlib.so.1.10.1",
    ],
)