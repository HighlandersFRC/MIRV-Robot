import os
from glob import glob
import re
from stdlib_list import stdlib_list
standard_libraries_list = stdlib_list("3.6")

PATH = '.'

result = [y for x in os.walk(PATH) for y in glob(os.path.join(x[0], '*.py'))]

print(result)

standard_libraries = {i: '' for i in standard_libraries_list}

version_mappings = {
    'opencv-contrib-python': "opencv-python"
}

versions = {}
package_versions = open('requirements_versions.txt').read().split('\n')
for v in package_versions:
    try:
        name = v.split('==')[0].lower()
        name = version_mappings.get(name, name)
        version = v.split('==')[1]
        versions[name] = version
    except:
        pass

print(versions)

all_imports = {}

# depthai covered by -e??
# neptunecontrib installed with neptune-client??
# pycocotools was not installed??
# visualization was not installed??
# ggadata was not installed??
# vtgdata was not installed??
# tablemanager was not installed??
# actionlib was not installed??
# ublox_msgs was not installed??

ROS_installed = ['geometry_msgs', 'std_msgs',
                 'sensor_msgs', 'nav_msgs', 'dse_msgs', 'rospy', 'roslib', 'ros_numpy', 'stdlib_list',
                 'catkin_pkg', 'rospy_tutorials', 'tf2_ros', 'tf2_geometry_msgs', 'actionlib', 'ublox_msgs', 'ggadata', 'vtgdata']
child_packages = ['cv2_bridge', 'cv_bridge', 'neptunecontrib']
directories_files = ['constants', 'helpful_functions_lib', 'pid', 'RoverInterface', 'RoverMacros',
                     'utils', '(', 'roverstate', 'lib', 'metrics', 'odometrystate', 'placement',
                     'rovermacros', 'roverinterface', 'mirv_control', 'robot_controller']
not_found = ['pycocotools', 'visualization', 'tablemanager']

ignored = []
ignored.extend(ROS_installed)
ignored.extend(child_packages)
ignored.extend(directories_files)
ignored.extend(not_found)

name_mappings = {
    'cv2': 'opencv-python',
    'socketio': 'python-socketio',
    'pil': 'pillow',
    'yaml': 'pyyaml',
    'serial': 'pyserial',
    'neptune': 'neptune-client',
    'sklearn': 'scikit-learn',
    'prefetch_generator': 'prefetch-generator',
    'importlib_metadata': 'importlib-metadata',
    'depthai': '-e git+https://github.com/luxonis/depthai-python.git@2e7110ea89ea18f3f4c688562ce1aa48549f79ce#egg=depthai',
    'torch': 'torch @ file:///home/nvidia/torch-1.10.0a0%2Bgit36449ea-cp36-cp36m-linux_aarch64.whl',
    'torchvision': 'torchvision @ file:///home/nvidia/torchvision-0.11.0a0%2Bfa347eb-cp36-cp36m-linux_aarch64.whl',
}

for file in result:
    try:
        contents = open(file, 'r').read()
        matches = re.findall('^import (.*?)\n', contents)
        matches.extend(re.findall('\nimport (.*?)\n', contents))
        matches.extend(re.findall('from (.*?) import', contents))
        for i in matches:
            name = i.strip()
            name = name.split(' as')[0]
            name = name.split('.')[0]
            name = name.split('__')[0]
            name = name.strip()
            name = name.lower()
            names = name.split(', ')
            if not name:
                continue
            for j in names:
                if j not in ignored and j not in standard_libraries:
                    all_imports[name_mappings.get(j, j)] = ['']
    except:
        pass

print(all_imports)

with open('requirements.txt', 'w') as f:
    for i in all_imports.keys():
        f.write(i + '==' + versions.get(i, '') + '\n')
