#!/bin/bash
# Para el robot inmediatamente — publica ceros a 20Hz por 3 segundos
source ~/ros2_ws/install/setup.bash
pkill -f drift_square_robot 2>/dev/null
timeout 3 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '{}' 2>/dev/null
echo "Robot detenido"
