from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['voltage_measurements_circuit'],
    package_dir={'': 'src'}
)
setup(**d)