#!/usr/bin/env python3
"""
scenario.launch.py — one command to bring up the EVALUATION DETECTOR LAYER for a
Task 6 scenario, once the sim + PX4 + commanders + fog_server are already running.

It starts, for the chosen mode:
  * the tier detector  (cloud_detector x N for cloud, victim_detector x N for
    local; nothing for fog — fog_server does detection itself)
  * decision_node      (so victims are dispatched -> completion time)
  * battery_simulator x N

It does NOT start: the simulator, PX4, commanders, fog_server (manual env/param
steps), or the metrics_collector. Run the COLLECTOR IN ITS OWN TERMINAL so it
saves reliably — a launch Ctrl-C can SIGTERM-kill bundled processes before they
save. The launch prints the exact collector command (with the right fault tag) on
startup.

Usage:
  ros2 launch ./scenario.launch.py mode:=fog   run_id:=run_01
  ros2 launch ./scenario.launch.py mode:=cloud run_id:=run_01
  ros2 launch ./scenario.launch.py mode:=local run_id:=run_01
  (optional: num_drones:=3 scenario:=medium)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FAULT_FOR_MODE = {"fog": "none", "cloud": "fog_down", "local": "fog_cloud_down"}


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    run_id = LaunchConfiguration("run_id").perform(context)
    scenario = LaunchConfiguration("scenario").perform(context)
    num_drones = int(LaunchConfiguration("num_drones").perform(context))
    fault = FAULT_FOR_MODE.get(mode, "none")

    nodes = []

    # ---- tier detector ----
    if mode == "cloud":
        for i in range(num_drones):
            nodes.append(Node(
                package="drone_node", executable="cloud_detector",
                name=f"cloud_detector_{i}",
                arguments=["--ros-args", "-p", f"instance:={i}"]))
    elif mode == "local":
        for i in range(num_drones):
            nodes.append(Node(
                package="drone_node", executable="victim_detector",
                name=f"victim_detector_{i}",
                arguments=["--ros-args", "-p", f"instance:={i}"]))
    # fog: fog_server (started separately) does detection -> no tier node here

    # ---- coordinator (victim dispatch -> completion time) ----
    nodes.append(Node(
        package="fog_node", executable="decision_node", name="decision_node",
        arguments=["--ros-args", "-p", f"num_drones:={num_drones}"]))

    # ---- battery sims ----
    for i in range(num_drones):
        nodes.append(Node(
            package="drone_node", executable="battery_simulator",
            name=f"battery_simulator_{i}",
            arguments=["--ros-args", "-p", f"drone_id:=drone{i}"]))

    # Tell the operator exactly how to start the collector in its own terminal.
    collector_cmd = (
        "RUN THE COLLECTOR IN ITS OWN TERMINAL:\n"
        f"  cd ~/ros2_ws && python3 evaluation/metrics_collector.py --ros-args "
        f"-p mode:={mode} -p scenario:={scenario} -p run_id:={run_id} "
        f"-p num_drones:={num_drones} -p fault:={fault}")
    return [LogInfo(msg=collector_cmd)] + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="fog",
                              description="fog | cloud | local"),
        DeclareLaunchArgument("run_id", default_value="run_01"),
        DeclareLaunchArgument("scenario", default_value="medium"),
        DeclareLaunchArgument("num_drones", default_value="3"),
        OpaqueFunction(function=launch_setup),
    ])