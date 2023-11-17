from setuptools import find_packages, setup

package_name = "mfrankic_5"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marin",
    maintainer_email="mfrankic@riteh.hr",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bb9_follower = mfrankic_5.bb9_follower:main",
        ],
    },
)
