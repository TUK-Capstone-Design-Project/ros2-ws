import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 경로 설정
    pkg_localizer = get_package_share_directory('lcode_localizer')
    pkg_description = get_package_share_directory('yahboomcar_description')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # 파일 경로
    nav2_params = os.path.join(pkg_localizer, 'params', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg_localizer, 'params', 'lcode_map.yaml')
    rviz_config = os.path.join(pkg_description, 'rviz', 'raspbotv2.rviz')
    urdf_file = os.path.join(pkg_description, 'urdf', 'Raspbot-V2.urdf')

    # URDF 데이터 직접 읽기
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # 2. Robot State Publisher (TF 구성의 핵심 - 반드시 최상단)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content
        }]
    )

    # 3. Joint State Publisher (바퀴 TF 생성용)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Gazebo 실행
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo_launch.py')
        )
    )

    # 5. L-Code Localizer (map -> base_link 연결 담당)
    localizer_node = Node(
        package='lcode_localizer',
        executable='lcode_localizer_node',
        name='lcode_localizer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. Nav2 Bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,           # 기본 인자
            'yaml_filename': map_yaml, # map_server 직접 주입용 (에러 해결 핵심)
            'use_sim_time': 'True',
            'params_file': nav2_params,
            'autostart': 'True',       # Lifecycle 자동 활성화
            'use_lifecycle_mgr': 'True'
        }.items()
    )

    # 7. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        gazebo_launch,
        localizer_node,
        nav2_launch,
        rviz_node
    ])
