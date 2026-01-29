from setuptools import setup, find_packages

setup(
    name="lerobot_ned2_plugins",
    version="0.0.2",
    packages=find_packages(),
    install_requires=[
        "lerobot",
        "pyniryo==1.2.3",
    ],
)
