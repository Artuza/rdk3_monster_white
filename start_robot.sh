#!/bin/bash
# Arranca el stack completo del robot desde la Jetson via SSH
# Uso: ./start_robot.sh [start|stop|status]

ROBOT_IP="192.168.8.88"
ROBOT_USER="root"
SESSION="robot"
LAUNCH_CMD="source /opt/ros/foxy/setup.bash && ros2 launch network_bridge robot_integration.launch.py"

cmd=${1:-start}

ssh_robot() {
    ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" "$@"
}

case "$cmd" in
  start)
    echo "Arrancando stack del robot en $ROBOT_IP..."
    ssh_robot "tmux new-session -d -s $SESSION '$LAUNCH_CMD' 2>/dev/null && echo 'Iniciado' || echo 'Ya estaba corriendo'"
    echo "Esperando que los topics estén disponibles (10s)..."
    sleep 10
    source ~/ros2_ws/install/setup.bash
    if ros2 topic list 2>/dev/null | grep -q vel_raw; then
      echo ""
      echo "Robot listo. Topics activos:"
      ros2 topic list 2>/dev/null | grep -E "vel_raw|imu/data|cmd_vel|odom|voltage"
    else
      echo "AVISO: topics aún no visibles — espera un poco más o revisa la conexión"
    fi
    ;;
  stop)
    echo "Deteniendo stack del robot..."
    ssh_robot "tmux kill-session -t $SESSION 2>/dev/null && echo 'Detenido' || echo 'No había sesión activa'"
    ;;
  status)
    echo "Sesiones tmux en el robot:"
    ssh_robot "tmux ls 2>/dev/null || echo 'Sin sesiones activas'"
    ;;
  *)
    echo "Uso: $0 [start|stop|status]"
    ;;
esac
