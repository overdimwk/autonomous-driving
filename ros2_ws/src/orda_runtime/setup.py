from glob import glob
from setuptools import find_packages, setup
setup(name='orda_runtime', version='1.0.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/orda_runtime']),
 ('share/orda_runtime',['package.xml']),
 ('share/orda_runtime/launch',glob('launch/*.launch.py')),
 ('share/orda_runtime/config',glob('config/*')),
 ('share/orda_runtime/models',glob('models/*'))],
 install_requires=['setuptools'], zip_safe=True,
 entry_points={'console_scripts':['supervise = orda_runtime.supervise:main']})
