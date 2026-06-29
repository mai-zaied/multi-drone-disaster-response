#!/usr/bin/env python3
"""
scenario.launch.py — one command to bring up the EVALUATION LAYER for a Task 6
scenario, once the sim + PX4 + commanders + fog_server are already running.

It starts, tagged for the chosen mode:
  * the tier detector  (cloud_detector x N for cloud, victim_detector x N for
    local; nothing for fog — fog_server does detection itself)
  * decision_node      (so victims are dispatched -> completion time)
  * battery_simulator x N
  * metrics_collector  (mode + fault tagged)

It does NOT start the simulator, PX4, commanders, or fog_server — those need the
manual env/param steps in the run guide. Start those first, then launch this.

Usage:
  ros2 launch ./scenario.launch.py mode:=fog   run_id:=run_01
  ros2 launch ./scenario.launch.py mode:=cloud run_id:=run_01
  ros2 launch ./scenario.launch.py mode:=local run_id:=run_01
  (optional: num_drones:=3 scenario:=medium collector_path:=evaluation/metrics_collector.py)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FAULT_FOR_MODE = {"fog": "none", "cloud": "fog_down", "local": "fog_cloud_down"}


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    run_id = LaunchConfiguration("run_id").perform(context)
    scenario = LaunchConfiguration("scenario").perform(context)
    num_drones = int(LaunchConfiguration("num_drones").perform(context))
    collector_path = LaunchConfiguration("collector_path").perform(context)
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

    # ---- metrics collector (plain script in evaluation/) ----
    nodes.append(ExecuteProcess(
        cmd=["python3", collector_path, "--ros-args",
             "-p", f"mode:={mode}", "-p", f"scenario:={scenario}",
             "-p", f"run_id:={run_id}", "-p", f"num_drones:={num_drones}",
             "-p", f"fault:={fault}"],
        output="screen"))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="fog",
                              description="fog | cloud | local"),
        DeclareLaunchArgument("run_id", default_value="run_01"),
        DeclareLaunchArgument("scenario", default_value="medium"),
        DeclareLaunchArgument("num_drones", default_value="3"),
        DeclareLaunchArgument("collector_path",
                              default_value="evaluation/metrics_collector.py"),
        OpaqueFunction(function=launch_setup),
    ])
