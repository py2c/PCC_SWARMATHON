## http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
# Do not run manually, catkin_make will run

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch from package.xml
setup_args = generate_distutils_setup(
        packages = ['mobility'],
        package_dir = {'': 'src'}
        )

setup(**setup_args)
