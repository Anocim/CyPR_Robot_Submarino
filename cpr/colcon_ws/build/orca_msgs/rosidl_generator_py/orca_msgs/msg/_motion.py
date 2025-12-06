# generated from rosidl_generator_py/resource/_idl.py.em
# with input from orca_msgs:msg/Motion.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Motion(type):
    """Metaclass of message 'Motion'."""

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
            module = import_type_support('orca_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'orca_msgs.msg.Motion')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motion
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motion
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motion
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motion
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motion

            from geometry_msgs.msg import Accel
            if Accel.__class__._TYPE_SUPPORT is None:
                Accel.__class__.__import_type_support__()

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

            from geometry_msgs.msg import Twist
            if Twist.__class__._TYPE_SUPPORT is None:
                Twist.__class__.__import_type_support__()

            from geometry_msgs.msg import Wrench
            if Wrench.__class__._TYPE_SUPPORT is None:
                Wrench.__class__.__import_type_support__()

            from orca_msgs.msg import Effort
            if Effort.__class__._TYPE_SUPPORT is None:
                Effort.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Motion(metaclass=Metaclass_Motion):
    """Message class 'Motion'."""

    __slots__ = [
        '_header',
        '_cmd_vel',
        '_dt',
        '_accel_model',
        '_accel_drag',
        '_accel_total',
        '_vel',
        '_pose',
        '_force',
        '_effort',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'cmd_vel': 'geometry_msgs/Twist',
        'dt': 'double',
        'accel_model': 'geometry_msgs/Accel',
        'accel_drag': 'geometry_msgs/Accel',
        'accel_total': 'geometry_msgs/Accel',
        'vel': 'geometry_msgs/Twist',
        'pose': 'geometry_msgs/Pose',
        'force': 'geometry_msgs/Wrench',
        'effort': 'orca_msgs/Effort',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Accel'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Accel'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Accel'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Wrench'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['orca_msgs', 'msg'], 'Effort'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from geometry_msgs.msg import Twist
        self.cmd_vel = kwargs.get('cmd_vel', Twist())
        self.dt = kwargs.get('dt', float())
        from geometry_msgs.msg import Accel
        self.accel_model = kwargs.get('accel_model', Accel())
        from geometry_msgs.msg import Accel
        self.accel_drag = kwargs.get('accel_drag', Accel())
        from geometry_msgs.msg import Accel
        self.accel_total = kwargs.get('accel_total', Accel())
        from geometry_msgs.msg import Twist
        self.vel = kwargs.get('vel', Twist())
        from geometry_msgs.msg import Pose
        self.pose = kwargs.get('pose', Pose())
        from geometry_msgs.msg import Wrench
        self.force = kwargs.get('force', Wrench())
        from orca_msgs.msg import Effort
        self.effort = kwargs.get('effort', Effort())

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
        if self.cmd_vel != other.cmd_vel:
            return False
        if self.dt != other.dt:
            return False
        if self.accel_model != other.accel_model:
            return False
        if self.accel_drag != other.accel_drag:
            return False
        if self.accel_total != other.accel_total:
            return False
        if self.vel != other.vel:
            return False
        if self.pose != other.pose:
            return False
        if self.force != other.force:
            return False
        if self.effort != other.effort:
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
    def cmd_vel(self):
        """Message field 'cmd_vel'."""
        return self._cmd_vel

    @cmd_vel.setter
    def cmd_vel(self, value):
        if __debug__:
            from geometry_msgs.msg import Twist
            assert \
                isinstance(value, Twist), \
                "The 'cmd_vel' field must be a sub message of type 'Twist'"
        self._cmd_vel = value

    @builtins.property
    def dt(self):
        """Message field 'dt'."""
        return self._dt

    @dt.setter
    def dt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dt' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dt' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dt = value

    @builtins.property
    def accel_model(self):
        """Message field 'accel_model'."""
        return self._accel_model

    @accel_model.setter
    def accel_model(self, value):
        if __debug__:
            from geometry_msgs.msg import Accel
            assert \
                isinstance(value, Accel), \
                "The 'accel_model' field must be a sub message of type 'Accel'"
        self._accel_model = value

    @builtins.property
    def accel_drag(self):
        """Message field 'accel_drag'."""
        return self._accel_drag

    @accel_drag.setter
    def accel_drag(self, value):
        if __debug__:
            from geometry_msgs.msg import Accel
            assert \
                isinstance(value, Accel), \
                "The 'accel_drag' field must be a sub message of type 'Accel'"
        self._accel_drag = value

    @builtins.property
    def accel_total(self):
        """Message field 'accel_total'."""
        return self._accel_total

    @accel_total.setter
    def accel_total(self, value):
        if __debug__:
            from geometry_msgs.msg import Accel
            assert \
                isinstance(value, Accel), \
                "The 'accel_total' field must be a sub message of type 'Accel'"
        self._accel_total = value

    @builtins.property
    def vel(self):
        """Message field 'vel'."""
        return self._vel

    @vel.setter
    def vel(self, value):
        if __debug__:
            from geometry_msgs.msg import Twist
            assert \
                isinstance(value, Twist), \
                "The 'vel' field must be a sub message of type 'Twist'"
        self._vel = value

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'pose' field must be a sub message of type 'Pose'"
        self._pose = value

    @builtins.property
    def force(self):
        """Message field 'force'."""
        return self._force

    @force.setter
    def force(self, value):
        if __debug__:
            from geometry_msgs.msg import Wrench
            assert \
                isinstance(value, Wrench), \
                "The 'force' field must be a sub message of type 'Wrench'"
        self._force = value

    @builtins.property
    def effort(self):
        """Message field 'effort'."""
        return self._effort

    @effort.setter
    def effort(self, value):
        if __debug__:
            from orca_msgs.msg import Effort
            assert \
                isinstance(value, Effort), \
                "The 'effort' field must be a sub message of type 'Effort'"
        self._effort = value
