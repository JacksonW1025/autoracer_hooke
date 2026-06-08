from setuptools import setup

package_name = "autoracer_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/pose_tf.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Autoracer Team",
    maintainer_email="autoracer@example.com",
    description="Small localization helpers for Autoracer Hooke.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "correlated_fixposition_noise = "
            "autoracer_localization.correlated_fixposition_noise:main",
            "fixposition_seed_filter = autoracer_localization.fixposition_seed_filter:main",
            "fixposition_odom_to_seed_pose = "
            "autoracer_localization.fixposition_odom_to_seed_pose:main",
            "ekf_feedback_gate = autoracer_localization.ekf_feedback_gate:main",
            "fixposition_startup_seed_gate = "
            "autoracer_localization.fixposition_startup_seed_gate:main",
            "ground_truth_initialpose_once = "
            "autoracer_localization.ground_truth_initialpose_once:main",
            "diagnostic_pose_reinitializer = "
            "autoracer_localization.diagnostic_pose_reinitializer:main",
            "startup_pose_initializer_once = "
            "autoracer_localization.startup_pose_initializer_once:main",
            "ndt_axis_seed_fuser = autoracer_localization.ndt_axis_seed_fuser:main",
            "ndt_initial_pose_predictor = autoracer_localization.ndt_initial_pose_predictor:main",
            "ndt_startup_helper = autoracer_localization.ndt_startup_helper:main",
            "pose_tf_broadcaster = autoracer_localization.pose_tf_broadcaster:main",
            "pointcloud_clock_publisher = "
            "autoracer_localization.pointcloud_clock_publisher:main",
            "vehicle_status_clock_publisher = "
            "autoracer_localization.vehicle_status_clock_publisher:main",
            "vehicle_status_to_twist_covariance = "
            "autoracer_localization.vehicle_status_to_twist_covariance:main",
        ],
    },
)
