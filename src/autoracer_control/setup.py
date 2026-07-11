from setuptools import setup

package_name = "autoracer_control"
launch_files = [
    "launch/control.launch.py",
    "launch/race_control.launch.py",
    "launch/race_control_bench.launch.py",
    "launch/control_closed_loop_bench.launch.py",
]
config_files = [
    "config/race_controller.param.yaml",
    "config/race_controller.closed_loop_candidate.param.yaml",
]
urbanroad_sim_config_files = [
    "config/urbanroad_sim/vehicle_info.param.yaml",
]

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", launch_files),
        (f"share/{package_name}/config", config_files),
        (f"share/{package_name}/config/urbanroad_sim", urbanroad_sim_config_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Autoracer Team",
    maintainer_email="autoracer@example.com",
    description="Pure pursuit controller for closed-track Autoracer driving.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pure_pursuit_controller = autoracer_control.pure_pursuit_controller:main",
            "race_bench_fixture_publisher = autoracer_control.race_bench_fixture_publisher:main",
            "race_bench_monitor = autoracer_control.race_bench_monitor:main",
            "control_closed_loop_fixture_publisher = autoracer_control.control_closed_loop_fixture_publisher:main",
            "virtual_chassis_node = autoracer_control.virtual_chassis_node:main",
            "control_closed_loop_monitor = autoracer_control.control_closed_loop_monitor:main",
        ],
    },
)
