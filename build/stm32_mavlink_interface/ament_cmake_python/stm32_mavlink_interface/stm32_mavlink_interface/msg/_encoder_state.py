# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stm32_mavlink_interface:msg/EncoderState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EncoderState(type):
    """Metaclass of message 'EncoderState'."""

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
                'stm32_mavlink_interface.msg.EncoderState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__encoder_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__encoder_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__encoder_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__encoder_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__encoder_state

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


class EncoderState(metaclass=Metaclass_EncoderState):
    """Message class 'EncoderState'."""

    __slots__ = [
        '_header',
        '_encoder_id',
        '_position',
        '_angle_rad',
        '_angle_deg',
        '_revolutions',
        '_z_detected',
        '_status',
        '_error_count',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'encoder_id': 'uint8',
        'position': 'int32',
        'angle_rad': 'float',
        'angle_deg': 'float',
        'revolutions': 'uint32',
        'z_detected': 'boolean',
        'status': 'uint8',
        'error_count': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
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
        self.encoder_id = kwargs.get('encoder_id', int())
        self.position = kwargs.get('position', int())
        self.angle_rad = kwargs.get('angle_rad', float())
        self.angle_deg = kwargs.get('angle_deg', float())
        self.revolutions = kwargs.get('revolutions', int())
        self.z_detected = kwargs.get('z_detected', bool())
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
        if self.encoder_id != other.encoder_id:
            return False
        if self.position != other.position:
            return False
        if self.angle_rad != other.angle_rad:
            return False
        if self.angle_deg != other.angle_deg:
            return False
        if self.revolutions != other.revolutions:
            return False
        if self.z_detected != other.z_detected:
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
    def encoder_id(self):
        """Message field 'encoder_id'."""
        return self._encoder_id

    @encoder_id.setter
    def encoder_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'encoder_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'encoder_id' field must be an unsigned integer in [0, 255]"
        self._encoder_id = value

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'position' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'position' field must be an integer in [-2147483648, 2147483647]"
        self._position = value

    @builtins.property
    def angle_rad(self):
        """Message field 'angle_rad'."""
        return self._angle_rad

    @angle_rad.setter
    def angle_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angle_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angle_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angle_rad = value

    @builtins.property
    def angle_deg(self):
        """Message field 'angle_deg'."""
        return self._angle_deg

    @angle_deg.setter
    def angle_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angle_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angle_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angle_deg = value

    @builtins.property
    def revolutions(self):
        """Message field 'revolutions'."""
        return self._revolutions

    @revolutions.setter
    def revolutions(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'revolutions' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'revolutions' field must be an unsigned integer in [0, 4294967295]"
        self._revolutions = value

    @builtins.property
    def z_detected(self):
        """Message field 'z_detected'."""
        return self._z_detected

    @z_detected.setter
    def z_detected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'z_detected' field must be of type 'bool'"
        self._z_detected = value

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
