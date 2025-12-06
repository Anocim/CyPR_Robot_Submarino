# generated from rosidl_generator_py/resource/_idl.py.em
# with input from orb_slam2_ros:msg/Status.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Status(type):
    """Metaclass of message 'Status'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STATE_SYSTEM_NOT_READY': -1,
        'STATE_NO_IMAGES_YET': 0,
        'STATE_NOT_INITIALIZED': 1,
        'STATE_OK': 2,
        'STATE_LOST': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('orb_slam2_ros')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'orb_slam2_ros.msg.Status')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__status

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'STATE_SYSTEM_NOT_READY': cls.__constants['STATE_SYSTEM_NOT_READY'],
            'STATE_NO_IMAGES_YET': cls.__constants['STATE_NO_IMAGES_YET'],
            'STATE_NOT_INITIALIZED': cls.__constants['STATE_NOT_INITIALIZED'],
            'STATE_OK': cls.__constants['STATE_OK'],
            'STATE_LOST': cls.__constants['STATE_LOST'],
        }

    @property
    def STATE_SYSTEM_NOT_READY(self):
        """Message constant 'STATE_SYSTEM_NOT_READY'."""
        return Metaclass_Status.__constants['STATE_SYSTEM_NOT_READY']

    @property
    def STATE_NO_IMAGES_YET(self):
        """Message constant 'STATE_NO_IMAGES_YET'."""
        return Metaclass_Status.__constants['STATE_NO_IMAGES_YET']

    @property
    def STATE_NOT_INITIALIZED(self):
        """Message constant 'STATE_NOT_INITIALIZED'."""
        return Metaclass_Status.__constants['STATE_NOT_INITIALIZED']

    @property
    def STATE_OK(self):
        """Message constant 'STATE_OK'."""
        return Metaclass_Status.__constants['STATE_OK']

    @property
    def STATE_LOST(self):
        """Message constant 'STATE_LOST'."""
        return Metaclass_Status.__constants['STATE_LOST']


class Status(metaclass=Metaclass_Status):
    """
    Message class 'Status'.

    Constants:
      STATE_SYSTEM_NOT_READY
      STATE_NO_IMAGES_YET
      STATE_NOT_INITIALIZED
      STATE_OK
      STATE_LOST
    """

    __slots__ = [
        '_header',
        '_left_stamp',
        '_right_stamp',
        '_state',
        '_only_tracking',
        '_keyframes',
        '_map_points',
        '_tracked',
        '_tracked_vo',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'left_stamp': 'builtin_interfaces/Time',
        'right_stamp': 'builtin_interfaces/Time',
        'state': 'int32',
        'only_tracking': 'boolean',
        'keyframes': 'int32',
        'map_points': 'int32',
        'tracked': 'int32',
        'tracked_vo': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from builtin_interfaces.msg import Time
        self.left_stamp = kwargs.get('left_stamp', Time())
        from builtin_interfaces.msg import Time
        self.right_stamp = kwargs.get('right_stamp', Time())
        self.state = kwargs.get('state', int())
        self.only_tracking = kwargs.get('only_tracking', bool())
        self.keyframes = kwargs.get('keyframes', int())
        self.map_points = kwargs.get('map_points', int())
        self.tracked = kwargs.get('tracked', int())
        self.tracked_vo = kwargs.get('tracked_vo', int())

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
        if self.header != other.header:
            return False
        if self.left_stamp != other.left_stamp:
            return False
        if self.right_stamp != other.right_stamp:
            return False
        if self.state != other.state:
            return False
        if self.only_tracking != other.only_tracking:
            return False
        if self.keyframes != other.keyframes:
            return False
        if self.map_points != other.map_points:
            return False
        if self.tracked != other.tracked:
            return False
        if self.tracked_vo != other.tracked_vo:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def left_stamp(self):
        """Message field 'left_stamp'."""
        return self._left_stamp

    @left_stamp.setter
    def left_stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'left_stamp' field must be a sub message of type 'Time'"
        self._left_stamp = value

    @builtins.property
    def right_stamp(self):
        """Message field 'right_stamp'."""
        return self._right_stamp

    @right_stamp.setter
    def right_stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'right_stamp' field must be a sub message of type 'Time'"
        self._right_stamp = value

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'state' field must be an integer in [-2147483648, 2147483647]"
        self._state = value

    @builtins.property
    def only_tracking(self):
        """Message field 'only_tracking'."""
        return self._only_tracking

    @only_tracking.setter
    def only_tracking(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'only_tracking' field must be of type 'bool'"
        self._only_tracking = value

    @builtins.property
    def keyframes(self):
        """Message field 'keyframes'."""
        return self._keyframes

    @keyframes.setter
    def keyframes(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'keyframes' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'keyframes' field must be an integer in [-2147483648, 2147483647]"
        self._keyframes = value

    @builtins.property
    def map_points(self):
        """Message field 'map_points'."""
        return self._map_points

    @map_points.setter
    def map_points(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'map_points' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'map_points' field must be an integer in [-2147483648, 2147483647]"
        self._map_points = value

    @builtins.property
    def tracked(self):
        """Message field 'tracked'."""
        return self._tracked

    @tracked.setter
    def tracked(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'tracked' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'tracked' field must be an integer in [-2147483648, 2147483647]"
        self._tracked = value

    @builtins.property
    def tracked_vo(self):
        """Message field 'tracked_vo'."""
        return self._tracked_vo

    @tracked_vo.setter
    def tracked_vo(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'tracked_vo' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'tracked_vo' field must be an integer in [-2147483648, 2147483647]"
        self._tracked_vo = value
