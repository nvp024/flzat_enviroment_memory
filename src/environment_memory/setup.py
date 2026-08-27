from glob import glob

from setuptools import find_packages, setup


package_name = "environment_memory"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Simulation Team",
    maintainer_email="simulation-team@example.com",
    description="Environment perception and persistent semantic-spatial memory",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "memory_build_manager = environment_memory.memory_build_manager:main",
            "memory_command_manager = environment_memory.memory_command_manager:main",
            "memory_manager = environment_memory.memory_manager:main",
            "memory_query_server = environment_memory.memory_query_server:main",
            "observation_manager = environment_memory.observation_manager:main",
            "query_memory = environment_memory.query_memory_cli:main",
            "semantic_observation_manager = "
            "environment_memory.semantic_observation_manager:main",
        ],
    },
)
