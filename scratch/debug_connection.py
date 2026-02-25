#!/usr/bin/env python3
"""
Diagnostic script to check ROS backend connection and services
"""
import socket
import time
import subprocess
import sys

def check_websocket_connection():
    """Check if ROS WebSocket server is accessible"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex(('localhost', 9090))
        sock.close()
        return result == 0
    except Exception as e:
        print(f"Error checking WebSocket connection: {e}")
        return False

def check_ros_processes():
    """Check for running ROS processes"""
    try:
        result = subprocess.run(['pgrep', '-f', 'ros'], capture_output=True, text=True)
        processes = result.stdout.strip().split('\n') if result.stdout.strip() else []
        return len(processes) > 0, processes
    except Exception as e:
        print(f"Error checking ROS processes: {e}")
        return False, []

def check_system_services():
    """Check if key system services are available"""
    services = [
        'session_manager',
        'rosbridge_websocket',
        'experiment_coordinator'
    ]

    available_services = []
    for service in services:
        try:
            result = subprocess.run(['pgrep', '-f', service], capture_output=True, text=True)
            if result.stdout.strip():
                available_services.append(service)
        except:
            pass

    return available_services

def main():
    print("=== Neurosimo Backend Diagnostic ===")
    print()

    # Check WebSocket connection
    print("1. Checking WebSocket connection (localhost:9090)...")
    ws_connected = check_websocket_connection()
    print(f"   Status: {'✓ Connected' if ws_connected else '✗ Not connected'}")
    print()

    # Check ROS processes
    print("2. Checking for ROS processes...")
    has_ros, processes = check_ros_processes()
    print(f"   Status: {'✓ ROS processes found' if has_ros else '✗ No ROS processes'}")
    if processes and processes[0]:
        print(f"   Processes ({len(processes)}):")
        for proc in processes[:10]:  # Show first 10
            if proc.strip():
                print(f"     - {proc}")
        if len(processes) > 10:
            print(f"     ... and {len(processes) - 10} more")
    print()

    # Check system services
    print("3. Checking key system services...")
    services = check_system_services()
    if services:
        print(f"   Found services: {', '.join(services)}")
    else:
        print("   No key services found")
    print()

    # Overall status
    print("=== Overall Status ===")
    all_good = ws_connected and has_ros and len(services) > 0
    print(f"Backend health: {'✓ Good' if all_good else '✗ Issues detected'}")

    if not all_good:
        print()
        print("Troubleshooting suggestions:")
        if not ws_connected:
            print("- Start the ROS backend system")
            print("- Check if rosbridge_websocket is running on port 9090")
        if not has_ros:
            print("- Launch the ROS system (check launch scripts)")
        if len(services) == 0:
            print("- Ensure session_manager and other core services are started")

if __name__ == '__main__':
    main()