from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
kwargs = generate_distutils_setup(
    packages=['pykitti'],
    package_dir={'': 'lib'}
)

setup(**kwargs)