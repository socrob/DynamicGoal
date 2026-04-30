from setuptools import find_packages, setup

package_name = "dynamic_goal2"

setup(
  name=package_name,
  version="0.0.0",
  packages=find_packages(exclude=["test"]),
  data_files=[
    ("share/ament_index/resource_index/packages",
      ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    ("share/" + package_name + "/launch", ["launch/gazebo.launch.py"]),
  ],
  install_requires=["setuptools"],
  zip_safe=True,
  maintainer="André Silva",
  maintainer_email="andre.s.silva@tecnico.ulisboa.pt",
  description="he Dynamic Goal package in ROS2.",
  license="Apache-2.0",
  tests_require=["pytest"],
  entry_points={
    "console_scripts": [
      "dynamic_goal2 = dynamic_goal2.dynamic_goal2:main",
      "spawn_object = dynamic_goal2.spawn_object:main",
      "test = dynamic_goal2.test:main"
    ],
  },
)
