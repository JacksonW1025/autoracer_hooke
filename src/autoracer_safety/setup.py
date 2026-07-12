from setuptools import setup

package_name = "autoracer_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/safety.launch.py",
                "launch/race_safety.launch.py",
                "launch/race_safety_bench.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config/urbanroad_sim",
            [
                "config/urbanroad_sim/vehicle_cmd_gate.safe.param.yaml",
                "config/urbanroad_sim/race_runtime.safe.param.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Autoracer Team",
    maintainer_email="autoracer@example.com",
    description="Compact race runtime management and final vehicle command gate.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "command_gate = autoracer_safety.command_gate:main",
            "race_runtime_manager = autoracer_safety.race_runtime_manager:main",
            "race_stack_fixture = autoracer_safety.race_stack_fixture:main",
            "race_stack_monitor = autoracer_safety.race_stack_monitor:main",
        ],
    },
)
