
Step 1 - ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False world:=/home/trsa2024/ros2_lab3/trsa_tb3_cone.world params_file:=./src/lab3_pkg/config/my_config.yaml

Step 2 - ros2 run goals_and_navigation detection

# ------------------ LAB 2 ---------------------

Step 1 - Launch Robot
    ros2 launch sam_bot_description display.launch.py

Step 2 - Launch Localization
    ros2 launch nav2_bringup localization_launch.py map:=./map/two_rooms_map.yaml use_sim_time:=true params_file:=/home/trsa2024/ros2_lab3/src/lab3_pkg/config/my_config.yaml

Step 3 - Launch Navigation
    ros2 launch nav2_bringup navigation_launch.py headless:=False map:=./map/two_rooms_map.yaml params_file:=/home/trsa2024/ros2_lab3/src/lab3_pkg/config/my_config.yaml

Step 4 - Launch Detection
    ros2 run goals_and_navigation detection
