export MAGIC_COOKIE="$(xauth list | head -n1 | sed s/:/$DISPLAY/g)"
echo $MAGIC_COOKIE
docker-compose -f "compose_ros2_vanilla.yaml" up --build