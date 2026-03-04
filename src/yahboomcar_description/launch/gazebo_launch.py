import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_name_in_model = 'Raspbot-V2'
    package_name = 'yahboomcar_description'
    urdf_name = "Raspbot-V2.urdf"

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)
    pkg_parent_path = os.path.join(pkg_share, '..')
    gazebo_models_path = os.path.join(pkg_share, 'models')
    world_file = os.path.join(pkg_share, 'worlds', 'lcode_sim.world')

    # 1. 환경 변수 설정 (중복 보장)
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=['/opt/ros/humble/lib']
    )
    
    # 2. Xacro 파싱
    robot_description_raw = xacro.process_file(urdf_model_path).toxml()

    # 3. 로봇 상태 퍼블리셔
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # [핵심 수정] Gazebo 실행 시 환경 변수를 직접 주입하여 실행
    # shell=True를 사용하여 소싱 후 실행하도록 합니다.
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c', 
            f'export GAZEBO_MODEL_PATH={pkg_parent_path}:{gazebo_models_path}:$GAZEBO_MODEL_PATH && '
            'export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH && '
            f'gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so {world_file}'
        ],
        output='screen'
    )

    # 4. 소환 노드 (Spawn)
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-topic', 'robot_description'],
        output='screen'
    )

    ground_sdf_path = os.path.join(pkg_share, 'models', 'lcode_ground', 'model.sdf')
    spawn_ground_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'lcode_ground', '-file', ground_sdf_path, '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_plugin_path,
        robot_state_publisher_node,
        start_gazebo_cmd,

        # 서비스를 찾지 못하는 문제를 방지하기 위해 대기 시간을 15초로 늘림
        TimerAction(
            period=15.0,
            actions=[spawn_robot_cmd, spawn_ground_cmd]
        )
    ])
