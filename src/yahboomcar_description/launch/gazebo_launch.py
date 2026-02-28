import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction  # TimerAction 추가
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_name_in_model = 'Raspbot-V2'
    package_name = 'yahboomcar_description'
    urdf_name = "Raspbot-V2.urdf"

    # 1. 경로 설정
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)

    # Gazebo 모델 경로 설정
    pkg_parent_path = os.path.join(pkg_share, '..')
    gazebo_models_path = os.path.join(pkg_share, 'models')

    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=['/opt/ros/humble/lib']
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[pkg_parent_path, ':', gazebo_models_path]
    )

    # 2. Xacro를 사용하여 URDF 파싱
    robot_description_raw = xacro.process_file(urdf_model_path).toxml()

    # 3. 노드 정의
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # Gazebo 실행 프로세스
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 로봇 소환 (Spawn)
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-topic', 'robot_description'],
        output='screen'
    )

    # 바닥 패턴 소환 (Spawn)
    ground_sdf_path = os.path.join(pkg_share, 'models', 'lcode_ground', 'model.sdf')
    spawn_ground_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'lcode_ground', '-file', ground_sdf_path, '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    # 4. 반환 (TimerAction 적용)
    return LaunchDescription([
        set_gazebo_plugin_path,
        set_gazebo_model_path,
        robot_state_publisher_node,
        start_gazebo_cmd,

        # Gazebo가 켜지고 서비스를 등록할 시간을 주기 위해 10초 대기 후 소환 실행
        TimerAction(
            period=10.0,
            actions=[spawn_robot_cmd, spawn_ground_cmd]
        )
    ])
