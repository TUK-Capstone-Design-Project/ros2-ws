from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 底盘驱动节点
    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver',
    )

    # follow_line 节点
    follow_line_node = Node(
        package='yahboomcar_astra',
        executable='follow_line', 
        name='follow_line_node',
        output='screen',
    )

    
    return LaunchDescription([
        follow_line_node,
        driver_node,
    ])
