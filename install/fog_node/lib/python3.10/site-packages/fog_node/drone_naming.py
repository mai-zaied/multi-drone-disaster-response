"""
drone_naming.py

DUPLICATED COPY — this file is intentionally identical to
drone_node/drone_node/drone_naming.py.

Single source of truth for translating between PX4 instance indices
and the derived names used across the system.

The naming convention is:
    PX4 instance 0   -> drone_id "drone0"   -> Gazebo model "x500_depth_0"   -> topic prefix ""
    PX4 instance N>0 -> drone_id "droneN"   -> Gazebo model "x500_depth_N"   -> topic prefix "/px4_N"
"""


def drone_id_for(instance: int) -> str:
    return f'drone{instance}'


def px4_namespace_for(instance: int) -> str:
    return '' if instance == 0 else f'/px4_{instance}'


def px4_topic_for(instance: int, topic_name: str) -> str:
    return f'{px4_namespace_for(instance)}/fmu/out/{topic_name}'


def gz_model_name_for(instance: int) -> str:
    return f'x500_depth_{instance}'
