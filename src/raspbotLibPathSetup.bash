source install/setup.bash
ros2 launch yahboomcar_description gazebo_launch.py robot_model:=raspbot

source install/setup.bash
ros2 run driver_node driver_node & 
ros2 launch lcode_localizer simulation.launch.py

source install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=True \
    map:=/home/hhh/colcon_ws/src/lcode_localizer/params/lcode_map.yaml \
    params_file:=/home/hhh/colcon_ws/src/lcode_localizer/params/nav2_params.yaml