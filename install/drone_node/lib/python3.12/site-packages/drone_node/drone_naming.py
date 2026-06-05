"""
drone_naming.py

Single source of truth for translating between PX4 instance indices
and the derived names used across the system.

The naming convention is:
    PX4 instance 0   -> drone_id "drone0"   -> Gazebo model "x500_depth_0"   -> topic prefix ""
    PX4 instance N>0 -> drone_id "droneN"   -> Gazebo model "x500_depth_N"   -> topic prefix "/px4_N"

This file is intentionally duplicated in fog_node/drone_naming.py so each
package is self-contained. The functions are tiny enough that the
duplication cost is far lower than the cost of sharing code across
ROS2 Python packages.
"""


def drone_id_for(instance: int) -> str:
    """drone0, drone1, drone2, ..."""
    return f'drone{instance}'


def px4_namespace_for(instance: int) -> str:
    """Empty for instance 0, '/px4_N' for instances >= 1 (PX4 convention)."""
    return '' if instance == 0 else f'/px4_{instance}'


def px4_topic_for(instance: int, topic_name: str) -> str:
    """E.g., px4_topic_for(2, 'vehicle_status_v1') -> '/px4_2/fmu/out/vehicle_status_v1'."""
    return f'{px4_namespace_for(instance)}/fmu/out/{topic_name}'


def gz_model_name_for(instance: int) -> str:
    """E.g., 'x500_depth_0', 'x500_depth_1', ..."""
    return f'x500_depth_{instance}'
