from setuptools import setup

package_name = "autoracer_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/planning.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Autoracer Team",
    maintainer_email="autoracer@example.com",
    description="Lanelet route and trajectory generation for closed-track driving.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lanelet_route_planner = autoracer_planning.lanelet_route_planner:main",
            "local_trajectory_planner = autoracer_planning.local_trajectory_planner:main",
            "route_goal_publisher = autoracer_planning.route_goal_publisher:main",
        ],
    },
)
