# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stm32_mavlink_interface:msg/RobomasterMotorConfig.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobomasterMotorConfig(type):
    """Metaclass of message 'RobomasterMotorConfig'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FAILSAFE_HOLD_POSITION': 0,
        'FAILSAFE_BRAKE': 1,
        'FAILSAFE_DISABLE_OUTPUT': 2,
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
                'stm32_mavlink_interface.msg.RobomasterMotorConfig')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robomaster_motor_config
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robomaster_motor_config
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robomaster_motor_config
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robomaster_motor_config
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robomaster_motor_config

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FAILSAFE_HOLD_POSITION': cls.__constants['FAILSAFE_HOLD_POSITION'],
            'FAILSAFE_BRAKE': cls.__constants['FAILSAFE_BRAKE'],
            'FAILSAFE_DISABLE_OUTPUT': cls.__constants['FAILSAFE_DISABLE_OUTPUT'],
        }

    @property
    def FAILSAFE_HOLD_POSITION(self):
        """Message constant 'FAILSAFE_HOLD_POSITION'."""
        return Metaclass_RobomasterMotorConfig.__constants['FAILSAFE_HOLD_POSITION']

    @property
    def FAILSAFE_BRAKE(self):
        """Message constant 'FAILSAFE_BRAKE'."""
        return Metaclass_RobomasterMotorConfig.__constants['FAILSAFE_BRAKE']

    @property
    def FAILSAFE_DISABLE_OUTPUT(self):
        """Message constant 'FAILSAFE_DISABLE_OUTPUT'."""
        return Metaclass_RobomasterMotorConfig.__constants['FAILSAFE_DISABLE_OUTPUT']


class RobomasterMotorConfig(metaclass=Metaclass_RobomasterMotorConfig):
    """
    Message class 'RobomasterMotorConfig'.

    Constants:
      FAILSAFE_HOLD_POSITION
      FAILSAFE_BRAKE
      FAILSAFE_DISABLE_OUTPUT
    """

    __slots__ = [
        '_header',
        '_motor_id',
        '_max_velocity_rps',
        '_max_acceleration_rps2',
        '_max_current_ma',
        '_min_current_ma',
        '_min_position_rad',
        '_max_position_rad',
        '_position_limits_enabled',
        '_watchdog_timeout_ms',
        '_max_temperature_celsius',
        '_failsafe_behavior',
        '_position_kp',
        '_position_ki',
        '_position_kd',
        '_velocity_kp',
        '_velocity_ki',
        '_velocity_kd',
        '_startup_position_rad',
        '_startup_mode',
        '_start_disabled',
        '_direction_inverted',
        '_position_offset_rad',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'motor_id': 'uint8',
        'max_velocity_rps': 'float',
        'max_acceleration_rps2': 'float',
        'max_current_ma': 'int16',
        'min_current_ma': 'int16',
        'min_position_rad': 'float',
        'max_position_rad': 'float',
        'position_limits_enabled': 'boolean',
        'watchdog_timeout_ms': 'uint32',
        'max_temperature_celsius': 'uint8',
        'failsafe_behavior': 'uint8',
        'position_kp': 'float',
        'position_ki': 'float',
        'position_kd': 'float',
        'velocity_kp': 'float',
        'velocity_ki': 'float',
        'velocity_kd': 'float',
        'startup_position_rad': 'float',
        'startup_mode': 'uint8',
        'start_disabled': 'boolean',
        'direction_inverted': 'boolean',
        'position_offset_rad': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.motor_id = kwargs.get('motor_id', int())
        self.max_velocity_rps = kwargs.get('max_velocity_rps', float())
        self.max_acceleration_rps2 = kwargs.get('max_acceleration_rps2', float())
        self.max_current_ma = kwargs.get('max_current_ma', int())
        self.min_current_ma = kwargs.get('min_current_ma', int())
        self.min_position_rad = kwargs.get('min_position_rad', float())
        self.max_position_rad = kwargs.get('max_position_rad', float())
        self.position_limits_enabled = kwargs.get('position_limits_enabled', bool())
        self.watchdog_timeout_ms = kwargs.get('watchdog_timeout_ms', int())
        self.max_temperature_celsius = kwargs.get('max_temperature_celsius', int())
        self.failsafe_behavior = kwargs.get('failsafe_behavior', int())
        self.position_kp = kwargs.get('position_kp', float())
        self.position_ki = kwargs.get('position_ki', float())
        self.position_kd = kwargs.get('position_kd', float())
        self.velocity_kp = kwargs.get('velocity_kp', float())
        self.velocity_ki = kwargs.get('velocity_ki', float())
        self.velocity_kd = kwargs.get('velocity_kd', float())
        self.startup_position_rad = kwargs.get('startup_position_rad', float())
        self.startup_mode = kwargs.get('startup_mode', int())
        self.start_disabled = kwargs.get('start_disabled', bool())
        self.direction_inverted = kwargs.get('direction_inverted', bool())
        self.position_offset_rad = kwargs.get('position_offset_rad', float())

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
        if self.motor_id != other.motor_id:
            return False
        if self.max_velocity_rps != other.max_velocity_rps:
            return False
        if self.max_acceleration_rps2 != other.max_acceleration_rps2:
            return False
        if self.max_current_ma != other.max_current_ma:
            return False
        if self.min_current_ma != other.min_current_ma:
            return False
        if self.min_position_rad != other.min_position_rad:
            return False
        if self.max_position_rad != other.max_position_rad:
            return False
        if self.position_limits_enabled != other.position_limits_enabled:
            return False
        if self.watchdog_timeout_ms != other.watchdog_timeout_ms:
            return False
        if self.max_temperature_celsius != other.max_temperature_celsius:
            return False
        if self.failsafe_behavior != other.failsafe_behavior:
            return False
        if self.position_kp != other.position_kp:
            return False
        if self.position_ki != other.position_ki:
            return False
        if self.position_kd != other.position_kd:
            return False
        if self.velocity_kp != other.velocity_kp:
            return False
        if self.velocity_ki != other.velocity_ki:
            return False
        if self.velocity_kd != other.velocity_kd:
            return False
        if self.startup_position_rad != other.startup_position_rad:
            return False
        if self.startup_mode != other.startup_mode:
            return False
        if self.start_disabled != other.start_disabled:
            return False
        if self.direction_inverted != other.direction_inverted:
            return False
        if self.position_offset_rad != other.position_offset_rad:
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
    def motor_id(self):
        """Message field 'motor_id'."""
        return self._motor_id

    @motor_id.setter
    def motor_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'motor_id' field must be an unsigned integer in [0, 255]"
        self._motor_id = value

    @builtins.property
    def max_velocity_rps(self):
        """Message field 'max_velocity_rps'."""
        return self._max_velocity_rps

    @max_velocity_rps.setter
    def max_velocity_rps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_velocity_rps' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'max_velocity_rps' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._max_velocity_rps = value

    @builtins.property
    def max_acceleration_rps2(self):
        """Message field 'max_acceleration_rps2'."""
        return self._max_acceleration_rps2

    @max_acceleration_rps2.setter
    def max_acceleration_rps2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_acceleration_rps2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'max_acceleration_rps2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._max_acceleration_rps2 = value

    @builtins.property
    def max_current_ma(self):
        """Message field 'max_current_ma'."""
        return self._max_current_ma

    @max_current_ma.setter
    def max_current_ma(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'max_current_ma' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'max_current_ma' field must be an integer in [-32768, 32767]"
        self._max_current_ma = value

    @builtins.property
    def min_current_ma(self):
        """Message field 'min_current_ma'."""
        return self._min_current_ma

    @min_current_ma.setter
    def min_current_ma(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'min_current_ma' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'min_current_ma' field must be an integer in [-32768, 32767]"
        self._min_current_ma = value

    @builtins.property
    def min_position_rad(self):
        """Message field 'min_position_rad'."""
        return self._min_position_rad

    @min_position_rad.setter
    def min_position_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'min_position_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'min_position_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._min_position_rad = value

    @builtins.property
    def max_position_rad(self):
        """Message field 'max_position_rad'."""
        return self._max_position_rad

    @max_position_rad.setter
    def max_position_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_position_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'max_position_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._max_position_rad = value

    @builtins.property
    def position_limits_enabled(self):
        """Message field 'position_limits_enabled'."""
        return self._position_limits_enabled

    @position_limits_enabled.setter
    def position_limits_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'position_limits_enabled' field must be of type 'bool'"
        self._position_limits_enabled = value

    @builtins.property
    def watchdog_timeout_ms(self):
        """Message field 'watchdog_timeout_ms'."""
        return self._watchdog_timeout_ms

    @watchdog_timeout_ms.setter
    def watchdog_timeout_ms(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'watchdog_timeout_ms' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'watchdog_timeout_ms' field must be an unsigned integer in [0, 4294967295]"
        self._watchdog_timeout_ms = value

    @builtins.property
    def max_temperature_celsius(self):
        """Message field 'max_temperature_celsius'."""
        return self._max_temperature_celsius

    @max_temperature_celsius.setter
    def max_temperature_celsius(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'max_temperature_celsius' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'max_temperature_celsius' field must be an unsigned integer in [0, 255]"
        self._max_temperature_celsius = value

    @builtins.property
    def failsafe_behavior(self):
        """Message field 'failsafe_behavior'."""
        return self._failsafe_behavior

    @failsafe_behavior.setter
    def failsafe_behavior(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'failsafe_behavior' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'failsafe_behavior' field must be an unsigned integer in [0, 255]"
        self._failsafe_behavior = value

    @builtins.property
    def position_kp(self):
        """Message field 'position_kp'."""
        return self._position_kp

    @position_kp.setter
    def position_kp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position_kp' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'position_kp' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._position_kp = value

    @builtins.property
    def position_ki(self):
        """Message field 'position_ki'."""
        return self._position_ki

    @position_ki.setter
    def position_ki(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position_ki' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'position_ki' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._position_ki = value

    @builtins.property
    def position_kd(self):
        """Message field 'position_kd'."""
        return self._position_kd

    @position_kd.setter
    def position_kd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position_kd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'position_kd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._position_kd = value

    @builtins.property
    def velocity_kp(self):
        """Message field 'velocity_kp'."""
        return self._velocity_kp

    @velocity_kp.setter
    def velocity_kp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_kp' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'velocity_kp' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._velocity_kp = value

    @builtins.property
    def velocity_ki(self):
        """Message field 'velocity_ki'."""
        return self._velocity_ki

    @velocity_ki.setter
    def velocity_ki(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_ki' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'velocity_ki' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._velocity_ki = value

    @builtins.property
    def velocity_kd(self):
        """Message field 'velocity_kd'."""
        return self._velocity_kd

    @velocity_kd.setter
    def velocity_kd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_kd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'velocity_kd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._velocity_kd = value

    @builtins.property
    def startup_position_rad(self):
        """Message field 'startup_position_rad'."""
        return self._startup_position_rad

    @startup_position_rad.setter
    def startup_position_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'startup_position_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'startup_position_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._startup_position_rad = value

    @builtins.property
    def startup_mode(self):
        """Message field 'startup_mode'."""
        return self._startup_mode

    @startup_mode.setter
    def startup_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'startup_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'startup_mode' field must be an unsigned integer in [0, 255]"
        self._startup_mode = value

    @builtins.property
    def start_disabled(self):
        """Message field 'start_disabled'."""
        return self._start_disabled

    @start_disabled.setter
    def start_disabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'start_disabled' field must be of type 'bool'"
        self._start_disabled = value

    @builtins.property
    def direction_inverted(self):
        """Message field 'direction_inverted'."""
        return self._direction_inverted

    @direction_inverted.setter
    def direction_inverted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'direction_inverted' field must be of type 'bool'"
        self._direction_inverted = value

    @builtins.property
    def position_offset_rad(self):
        """Message field 'position_offset_rad'."""
        return self._position_offset_rad

    @position_offset_rad.setter
    def position_offset_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position_offset_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'position_offset_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._position_offset_rad = value
