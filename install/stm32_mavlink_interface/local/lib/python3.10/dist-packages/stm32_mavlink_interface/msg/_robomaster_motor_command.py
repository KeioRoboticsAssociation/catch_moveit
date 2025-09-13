# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stm32_mavlink_interface:msg/RobomasterMotorCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobomasterMotorCommand(type):
    """Metaclass of message 'RobomasterMotorCommand'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CONTROL_MODE_CURRENT': 0,
        'CONTROL_MODE_VELOCITY': 1,
        'CONTROL_MODE_POSITION': 2,
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
                'stm32_mavlink_interface.msg.RobomasterMotorCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robomaster_motor_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robomaster_motor_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robomaster_motor_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robomaster_motor_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robomaster_motor_command

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CONTROL_MODE_CURRENT': cls.__constants['CONTROL_MODE_CURRENT'],
            'CONTROL_MODE_VELOCITY': cls.__constants['CONTROL_MODE_VELOCITY'],
            'CONTROL_MODE_POSITION': cls.__constants['CONTROL_MODE_POSITION'],
        }

    @property
    def CONTROL_MODE_CURRENT(self):
        """Message constant 'CONTROL_MODE_CURRENT'."""
        return Metaclass_RobomasterMotorCommand.__constants['CONTROL_MODE_CURRENT']

    @property
    def CONTROL_MODE_VELOCITY(self):
        """Message constant 'CONTROL_MODE_VELOCITY'."""
        return Metaclass_RobomasterMotorCommand.__constants['CONTROL_MODE_VELOCITY']

    @property
    def CONTROL_MODE_POSITION(self):
        """Message constant 'CONTROL_MODE_POSITION'."""
        return Metaclass_RobomasterMotorCommand.__constants['CONTROL_MODE_POSITION']


class RobomasterMotorCommand(metaclass=Metaclass_RobomasterMotorCommand):
    """
    Message class 'RobomasterMotorCommand'.

    Constants:
      CONTROL_MODE_CURRENT
      CONTROL_MODE_VELOCITY
      CONTROL_MODE_POSITION
    """

    __slots__ = [
        '_header',
        '_motor_id',
        '_control_mode',
        '_target_current_ma',
        '_target_velocity_rps',
        '_target_position_rad',
        '_enabled',
        '_reset_watchdog',
        '_emergency_stop',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'motor_id': 'uint8',
        'control_mode': 'uint8',
        'target_current_ma': 'int16',
        'target_velocity_rps': 'float',
        'target_position_rad': 'float',
        'enabled': 'boolean',
        'reset_watchdog': 'boolean',
        'emergency_stop': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.motor_id = kwargs.get('motor_id', int())
        self.control_mode = kwargs.get('control_mode', int())
        self.target_current_ma = kwargs.get('target_current_ma', int())
        self.target_velocity_rps = kwargs.get('target_velocity_rps', float())
        self.target_position_rad = kwargs.get('target_position_rad', float())
        self.enabled = kwargs.get('enabled', bool())
        self.reset_watchdog = kwargs.get('reset_watchdog', bool())
        self.emergency_stop = kwargs.get('emergency_stop', bool())

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
        if self.control_mode != other.control_mode:
            return False
        if self.target_current_ma != other.target_current_ma:
            return False
        if self.target_velocity_rps != other.target_velocity_rps:
            return False
        if self.target_position_rad != other.target_position_rad:
            return False
        if self.enabled != other.enabled:
            return False
        if self.reset_watchdog != other.reset_watchdog:
            return False
        if self.emergency_stop != other.emergency_stop:
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
    def control_mode(self):
        """Message field 'control_mode'."""
        return self._control_mode

    @control_mode.setter
    def control_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'control_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'control_mode' field must be an unsigned integer in [0, 255]"
        self._control_mode = value

    @builtins.property
    def target_current_ma(self):
        """Message field 'target_current_ma'."""
        return self._target_current_ma

    @target_current_ma.setter
    def target_current_ma(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'target_current_ma' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'target_current_ma' field must be an integer in [-32768, 32767]"
        self._target_current_ma = value

    @builtins.property
    def target_velocity_rps(self):
        """Message field 'target_velocity_rps'."""
        return self._target_velocity_rps

    @target_velocity_rps.setter
    def target_velocity_rps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_velocity_rps' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target_velocity_rps' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target_velocity_rps = value

    @builtins.property
    def target_position_rad(self):
        """Message field 'target_position_rad'."""
        return self._target_position_rad

    @target_position_rad.setter
    def target_position_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_position_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target_position_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target_position_rad = value

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
    def reset_watchdog(self):
        """Message field 'reset_watchdog'."""
        return self._reset_watchdog

    @reset_watchdog.setter
    def reset_watchdog(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'reset_watchdog' field must be of type 'bool'"
        self._reset_watchdog = value

    @builtins.property
    def emergency_stop(self):
        """Message field 'emergency_stop'."""
        return self._emergency_stop

    @emergency_stop.setter
    def emergency_stop(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'emergency_stop' field must be of type 'bool'"
        self._emergency_stop = value
