from glob import glob
from setuptools import find_packages, setup
setup(name='hanyang_autonomy', version='1.0.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/hanyang_autonomy']),
 ('share/hanyang_autonomy',['package.xml']),
 ('share/hanyang_autonomy/launch',glob('launch/*.launch.py')),
 ('share/hanyang_autonomy/config',glob('config/*')),
 ('share/hanyang_autonomy/models',glob('models/*'))],
 install_requires=['setuptools'], zip_safe=True,
 entry_points={'console_scripts':['autonomy = hanyang_autonomy.main_node:main']})
