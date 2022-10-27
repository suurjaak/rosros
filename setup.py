# -*- coding: utf-8 -*-
"""
Setup.py for rosros.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2 Python API.
Released under the BSD License.

@author      Erki Suurjaak
@created     11.02.2022
@modified    23.10.2022
------------------------------------------------------------------------------
"""
import os
import re
import setuptools
try: from catkin_pkg.python_setup import generate_distutils_setup
except ImportError: generate_distutils_setup = None


PACKAGE = "rosros"


def readfile(path):
    """Returns contents of path, relative to current file."""
    root = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(root, path)) as f: return f.read()

def get_description():
    """Returns package description from README."""
    LINK_RGX = r"\[([^\]]+)\]\(([^\)]+)\)"  # 1: content in [], 2: content in ()
    linkify = lambda s: "#" + re.sub(r"[^\w -]", "", s).lower().replace(" ", "-")
    # Unwrap local links like [Page link](#page-link) and [LICENSE.md](LICENSE.md)
    repl = lambda m: m.group(1 if m.group(2) in (m.group(1), linkify(m.group(1))) else 0)
    return re.sub(LINK_RGX, repl, readfile("README.md"))

def get_version():
    """Returns package current version number from source code."""
    VERSION_RGX = r'__version__\s*\=\s*\"*([^\n\"]+)'
    content = readfile(os.path.join("src", PACKAGE, "__init__.py"))
    match = re.search(VERSION_RGX, content)
    return match.group(1).strip() if match else None


common_args = dict(
    install_requires = ["pyyaml"],
    package_dir      = {"": "src"},
    packages         = [PACKAGE],
)
version_args = dict(
    data_files      = [
        (os.path.join("share", "ament_index", "resource_index", "packages"),
         [os.path.join("resource", PACKAGE)]),
        (os.path.join("share", PACKAGE), ["package.xml"]),
    ],
    tests_require   = ["pytest", "pytest-forked"],
) if "2" == os.getenv("ROS_VERSION") else {}


setup_args = generate_distutils_setup(  # fetch values from package.xml
) if generate_distutils_setup and "1" == os.getenv("ROS_VERSION") else dict(  # Normal pip setup
    name                 = PACKAGE,
    version              = get_version(),

    description          = "Simple unified interface to ROS1 / ROS2 Python API",
    url                  = "https://github.com/suurjaak/" + PACKAGE,
    author               = "Erki Suurjaak",
    author_email         = "erki@lap.ee",
    license              = "BSD",
    platforms            = ["any"],
    keywords             = "ROS ROS1 ROS2",
    python_requires      = ">=3",

    include_package_data = True, # Use MANIFEST.in for data files
    classifiers          = [
        "Development Status :: 4 - Beta",
        "License :: OSI Approved :: BSD License",
        "Intended Audience :: Developers",
        "Operating System :: POSIX",
        "Topic :: Scientific/Engineering",
        "Topic :: Utilities",
        "Programming Language :: Python :: 3",
    ],

    long_description_content_type = "text/markdown",
    long_description              = get_description(),
)
setuptools.setup(**dict(common_args, **dict(setup_args, **version_args)))
