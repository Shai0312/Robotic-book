#!/usr/bin/env python3
"""
Setup script for VLA (Vision-Language-Action) module
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

with open("requirements.txt", "r", encoding="utf-8") as fh:
    requirements = [line.strip() for line in fh if line.strip() and not line.startswith("#")]

setup(
    name="module4_vla",
    version="1.0.0",
    author="AI and Robotics Student",
    author_email="student@example.com",
    description="Vision-Language-Action (VLA) module for robotics applications",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages(),
    install_requires=requirements,
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    keywords="robotics, ai, nlp, computer-vision, ros",
    project_urls={
        "Source": "https://github.com/cz-3/Robotic",
        "Tracker": "https://github.com/cz-3/Robotic/issues",
    },
)