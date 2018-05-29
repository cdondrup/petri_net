## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rpn_common', 'rpn_gen', 'rpn_ros_interface', 'rpn_kb', 'rpn_execution', 'rpn_actions'],
    package_dir={'': 'src'})

setup(**setup_args)
