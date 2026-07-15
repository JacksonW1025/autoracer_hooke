from setuptools import setup

package_name = "autoracer_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/fixed_course_planning.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Autoracer Team",
    maintainer_email="autoracer@example.com",
    description="Validated fixed-course trajectory generation for closed-track driving.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "local_trajectory_planner = autoracer_planning.local_trajectory_planner:main",
            "fixed_course_publisher = autoracer_planning.fixed_course_publisher:main",
            "build_fixed_course = autoracer_planning.fixed_course:main",
            "build_map_manifest = autoracer_planning.map_manifest:main",
        ],
    },
)
