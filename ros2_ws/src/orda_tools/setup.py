from glob import glob
from setuptools import find_packages, setup
setup(name='orda_tools', version='1.0.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/orda_tools']),
 ('share/orda_tools',['package.xml']),
 ('share/orda_tools/launch',glob('launch/*.launch.py')),
 ('share/orda_tools/config',glob('config/*')),
 ('share/orda_tools/models',glob('models/*'))],
 install_requires=['setuptools'], zip_safe=True,
 entry_points={'console_scripts':['topic_monitor = orda_tools.topic_monitor:main',
     'sensor_snapshot = orda_tools.sensor_snapshot:main']})
