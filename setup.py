from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['health_node'],
    scripts=['scripts/health_node_server_script', 'scripts/health_node_client_script'],
    package_dir={'': 'src'}
)

setup(**d)