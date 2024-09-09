from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=['siemens_plc'],
    package_dir={'': 'src'}
)

setup(**d)
