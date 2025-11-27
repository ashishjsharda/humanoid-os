"""
HumanoidOS - An open-source operating system for humanoid robots
"""

from setuptools import setup, find_packages
import os

# Read README for long description
with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

# Read requirements
with open("requirements.txt", "r", encoding="utf-8") as fh:
    requirements = [line.strip() for line in fh if line.strip() and not line.startswith("#")]

setup(
    name="humanoid-os",
    version="0.1.0",
    author="Ashish Sharda",
    author_email="ashish@example.com",
    description="An open-source operating system for humanoid robots",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ashishsharda/humanoid-os",
    project_urls={
        "Bug Tracker": "https://github.com/ashishsharda/humanoid-os/issues",
        "Documentation": "https://github.com/ashishsharda/humanoid-os/tree/main/docs",
        "Source Code": "https://github.com/ashishsharda/humanoid-os",
    },
    packages=find_packages(exclude=["tests", "examples", "docs"]),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Robotics",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.8",
    install_requires=requirements,
    extras_require={
        "dev": [
            "pytest>=7.4.0",
            "pytest-cov>=4.1.0",
            "black>=23.0.0",
            "mypy>=1.4.0",
            "pylint>=2.17.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "humanoid-balance=examples.balance_demo:main",
            "humanoid-walking=examples.walking_demo:main",
        ],
    },
    include_package_data=True,
    zip_safe=False,
    keywords=[
        "robotics",
        "humanoid",
        "bipedal",
        "walking",
        "balance",
        "control",
        "simulation",
        "pybullet",
        "zero-moment-point",
        "zmp",
    ],
)
