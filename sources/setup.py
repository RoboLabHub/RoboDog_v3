from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['robodog_v3'],
    #scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)
