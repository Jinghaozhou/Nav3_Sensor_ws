from setuptools import setup
from glob import glob
from os.path import isfile
setup(name='mid360_bringup', version='0.1.0', packages=['mid360_bringup'],
 data_files=[('share/ament_index/resource_index/packages',['resource/mid360_bringup']),
 ('share/mid360_bringup',['package.xml','README.md']),
 *[('share/mid360_bringup/'+d,[f for f in glob(d+'/*') if isfile(f)]) for d in ['launch','config','rviz']]],
 install_requires=['setuptools'], zip_safe=True, maintainer='Jinghao',
 maintainer_email='zhoujinghao188@gmail.com', description='MID360 unified mapping bringup',
 license='Apache-2.0', entry_points={'console_scripts':[
 'livox_monitor = mid360_bringup.livox_monitor:main',
 'local_map_window = mid360_bringup.local_map_window:main',
 'save_map = mid360_bringup.save_map:main']})
