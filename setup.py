import os
import setuptools

with open("requirements.txt") as f:
    requirements = f.read().splitlines()

setuptools.setup(name='pgo3d',
version='0.1',
description='Pose graph visualization in Open3D',
url='#',
author='keevindoherty',
install_requires=requirements,
author_email='kdoherty@mit.edu',
packages=setuptools.find_packages(),
zip_safe=False)
