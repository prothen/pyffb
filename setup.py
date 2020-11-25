#!/usr/bin/env python
# -*- coding: utf-8 -*-
import setuptools


with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pyffb",
    version="0.0.1",
    author="Philipp Rothenhäusler",
    author_email="philipp.rothenhaeusler@gmail.com",
    description="Python interface for force feedback joystick.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/prothen/pyffb.git",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: Other/Proprietary License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
