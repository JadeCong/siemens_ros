from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=['modbus'],
    package_dir={'': 'src'},
    requires=['rospy', 'dynamic_reconfigure']
)

setup(**d)
