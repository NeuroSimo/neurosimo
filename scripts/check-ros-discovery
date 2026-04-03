#!/bin/bash
#
# check_ros_discovery.sh
#
# Verify the ROS_AUTOMATIC_DISCOVERY_RANGE setting across all services.
# Useful for confirming that localhost-only discovery is consistently applied
# and that no service accidentally exposes DDS traffic to the network.
#
# Checks:
#   - Current shell environment
#   - .env files for both NeuroSimo and mTMS
#   - All running Docker containers
#   - Systemd services that use ROS

echo "=== Shell environment ==="
echo "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-<unset>}"

echo ""
echo "=== .env files ==="
found_env=false
for env_file in ~/*/.env; do
    if [ -f "$env_file" ] && grep -q 'ROS_AUTOMATIC_DISCOVERY_RANGE' "$env_file" 2>/dev/null; then
        val=$(grep ROS_AUTOMATIC_DISCOVERY_RANGE "$env_file" | grep -v '^#' | cut -d= -f2)
        echo "$env_file: ${val:-<not set>}"
        found_env=true
    fi
done
if [ "$found_env" = false ]; then
    echo "(no .env files with ROS_AUTOMATIC_DISCOVERY_RANGE found)"
fi

echo ""
echo "=== Running containers ==="
for container in $(docker ps --format '{{.Names}}'); do
    val=$(docker exec "$container" printenv ROS_AUTOMATIC_DISCOVERY_RANGE 2>/dev/null)
    echo "$container: ${val:-<unset>}"
done

echo ""
echo "=== Systemd services ==="
for unit in $(systemctl list-units --type=service --state=running --no-legend | awk '{print $1}'); do
    val=$(systemctl show "$unit" -p Environment --value 2>/dev/null | grep -o 'ROS_AUTOMATIC_DISCOVERY_RANGE=[^ ]*' | cut -d= -f2)
    if [ -n "$val" ]; then
        echo "$unit: $val"
    fi
done
if [ -z "$(systemctl list-units --type=service --state=running --no-legend | awk '{print $1}' | while read unit; do
    systemctl show "$unit" -p Environment --value 2>/dev/null | grep -q 'ROS_AUTOMATIC_DISCOVERY_RANGE' && echo found
done)" ]; then
    echo "(no running systemd services with ROS_AUTOMATIC_DISCOVERY_RANGE found)"
fi
