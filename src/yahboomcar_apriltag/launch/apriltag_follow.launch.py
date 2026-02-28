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

    # USB 摄像头节点
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        # remappings=[
        #     ('image_raw', 'camera/image_raw'),
        #     ('camera_info', 'camera/camera_info'),
        # ],
    )

    # apriltag_follow 节点
    apriltag_follow_node = Node(
        package='yahboomcar_apriltag',
        executable='apriltag_follow', 
        name='apriltag_follow_node',
        output='screen',
    )

    
    return LaunchDescription([
        apriltag_follow_node,
        usb_cam_node,
        driver_node,
    ])
