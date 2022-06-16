from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['spot_driver', 'cv2'], #, 'cv_bridge'
    scripts=['scripts/spot_ros'],
    package_dir={'': 'src'}
)

setup(**d)