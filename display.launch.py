import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
  pkgPath = launch_ros.substitutions.FindPackageShare(package='pkg_urdf').find('pkg_urdf')
  urdfModelPath = os.path.join(pkgPath,
