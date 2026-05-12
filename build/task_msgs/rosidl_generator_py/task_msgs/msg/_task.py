# generated from rosidl_generator_py/resource/_idl.py.em
# with input from task_msgs:msg/Task.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Task(type):
    """Metaclass of message 'Task'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('task_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'task_msgs.msg.Task')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__task
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__task
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__task
            cls._TYPE_SUPPORT = module.type_support_msg__msg__task
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__task

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Task(metaclass=Metaclass_Task):
    """Message class 'Task'."""

    __slots__ = [
        '_task_id',
        '_task_type',
        '_drone_id',
        '_timestamp',
        '_priority',
        '_payload',
    ]

    _fields_and_field_types = {
        'task_id': 'string',
        'task_type': 'string',
        'drone_id': 'string',
        'timestamp': 'builtin_interfaces/Time',
        'priority': 'uint8',
        'payload': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.task_id = kwargs.get('task_id', str())
        self.task_type = kwargs.get('task_type', str())
        self.drone_id = kwargs.get('drone_id', str())
        from builtin_interfaces.msg import Time
        self.timestamp = kwargs.get('timestamp', Time())
        self.priority = kwargs.get('priority', int())
        self.payload = kwargs.get('payload', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.task_id != other.task_id:
            return False
        if self.task_type != other.task_type:
            return False
        if self.drone_id != other.drone_id:
            return False
        if self.timestamp != other.timestamp:
            return False
        if self.priority != other.priority:
            return False
        if self.payload != other.payload:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def task_id(self):
        """Message field 'task_id'."""
        return self._task_id

    @task_id.setter
    def task_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'task_id' field must be of type 'str'"
        self._task_id = value

    @builtins.property
    def task_type(self):
        """Message field 'task_type'."""
        return self._task_type

    @task_type.setter
    def task_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'task_type' field must be of type 'str'"
        self._task_type = value

    @builtins.property
    def drone_id(self):
        """Message field 'drone_id'."""
        return self._drone_id

    @drone_id.setter
    def drone_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'drone_id' field must be of type 'str'"
        self._drone_id = value

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'timestamp' field must be a sub message of type 'Time'"
        self._timestamp = value

    @builtins.property
    def priority(self):
        """Message field 'priority'."""
        return self._priority

    @priority.setter
    def priority(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'priority' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'priority' field must be an unsigned integer in [0, 255]"
        self._priority = value

    @builtins.property
    def payload(self):
        """Message field 'payload'."""
        return self._payload

    @payload.setter
    def payload(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'payload' field must be of type 'str'"
        self._payload = value
