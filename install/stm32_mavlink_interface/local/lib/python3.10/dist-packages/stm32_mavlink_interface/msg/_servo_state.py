# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stm32_mavlink_interface:msg/ServoState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ServoState(type):
    """Metaclass of message 'ServoState'."""

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
            module = import_type_support('stm32_mavlink_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stm32_mavlink_interface.msg.ServoState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__servo_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__servo_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__servo_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__servo_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__servo_state

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


class ServoState(metaclass=Metaclass_ServoState):
    """Message class 'ServoState'."""

    __slots__ = [
        '_header',
        '_servo_id',
        '_current_angle_deg',
        '_target_angle_deg',
        '_pulse_us',
        '_enabled',
        '_status',
        '_error_count',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'servo_id': 'uint8',
        'current_angle_deg': 'float',
        'target_angle_deg': 'float',
        'pulse_us': 'uint16',
        'enabled': 'boolean',
        'status': 'uint8',
        'error_count': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.servo_id = kwargs.get('servo_id', int())
        self.current_angle_deg = kwargs.get('current_angle_deg', float())
        self.target_angle_deg = kwargs.get('target_angle_deg', float())
        self.pulse_us = kwargs.get('pulse_us', int())
        self.enabled = kwargs.get('enabled', bool())
        self.status = kwargs.get('status', int())
        self.error_count = kwargs.get('error_count', int())

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
        if self.servo_id != other.servo_id:
            return False
        if self.current_angle_deg != other.current_angle_deg:
            return False
        if self.target_angle_deg != other.target_angle_deg:
            return False
        if self.pulse_us != other.pulse_us:
            return False
        if self.enabled != other.enabled:
            return False
        if self.status != other.status:
            return False
        if self.error_count != other.error_count:
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
    def servo_id(self):
        """Message field 'servo_id'."""
        return self._servo_id

    @servo_id.setter
    def servo_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'servo_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'servo_id' field must be an unsigned integer in [0, 255]"
        self._servo_id = value

    @builtins.property
    def current_angle_deg(self):
        """Message field 'current_angle_deg'."""
        return self._current_angle_deg

    @current_angle_deg.setter
    def current_angle_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_angle_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current_angle_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current_angle_deg = value

    @builtins.property
    def target_angle_deg(self):
        """Message field 'target_angle_deg'."""
        return self._target_angle_deg

    @target_angle_deg.setter
    def target_angle_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_angle_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target_angle_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target_angle_deg = value

    @builtins.property
    def pulse_us(self):
        """Message field 'pulse_us'."""
        return self._pulse_us

    @pulse_us.setter
    def pulse_us(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pulse_us' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'pulse_us' field must be an unsigned integer in [0, 65535]"
        self._pulse_us = value

    @builtins.property
    def enabled(self):
        """Message field 'enabled'."""
        return self._enabled

    @enabled.setter
    def enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'enabled' field must be of type 'bool'"
        self._enabled = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'status' field must be an unsigned integer in [0, 255]"
        self._status = value

    @builtins.property
    def error_count(self):
        """Message field 'error_count'."""
        return self._error_count

    @error_count.setter
    def error_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'error_count' field must be an unsigned integer in [0, 4294967295]"
        self._error_count = value
