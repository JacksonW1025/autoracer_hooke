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
            "fixposition_seed_filter = autoracer_localization.fixposition_seed_filter:main",
            "ndt_initial_pose_predictor = autoracer_localization.ndt_initial_pose_predictor:main",
            "ndt_startup_helper = autoracer_localization.ndt_startup_helper:main",
            "pose_tf_broadcaster = autoracer_localization.pose_tf_broadcaster:main",
        ],
    },
)
