#Terminal 0
cd ~/microros_ws
source install/local_setup.zsh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

#terminal 1
source /opt/ros/humble/setup.zsh
cd avoider_ros2
ros2 launch ./avoider.launch.py

atau

python3 avoider_brain.py
python3 desktop.py

#terminal 2 opsional buat liat parameter dan telemetry
source /opt/ros/humble/setup.zsh
ros2 topic list
ros2 topic echo /avoider/telemetry
ros2 topic hz /avoider/telemetry         # harus sekitar 40 Hz

ros2 param list
ros2 param list /avoider_brain
