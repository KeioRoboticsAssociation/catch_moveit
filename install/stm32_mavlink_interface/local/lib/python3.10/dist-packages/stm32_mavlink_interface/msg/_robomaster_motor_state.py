# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stm32_mavlink_interface:msg/RobomasterMotorState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobomasterMotorState(type):
    """Metaclass of message 'RobomasterMotorState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STATUS_OK': 0,
        'STATUS_NOT_INITIALIZED': 1,
        'STATUS_CAN_ERROR': 2,
        'STATUS_OUT_OF_RANGE': 3,
        'STATUS_TIMEOUT': 4,
        'STATUS_CONFIG_ERROR': 5,
        'STATUS_OVERHEAT': 6,
        'STATUS_OVERCURRENT': 7,
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
                'stm32_mavlink_interface.msg.RobomasterMotorState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robomaster_motor_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robomaster_motor_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robomaster_motor_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robomaster_motor_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robomaster_motor_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'STATUS_OK': cls.__constants['STATUS_OK'],
            'STATUS_NOT_INITIALIZED': cls.__constants['STATUS_NOT_INITIALIZED'],
            'STATUS_CAN_ERROR': cls.__constants['STATUS_CAN_ERROR'],
            'STATUS_OUT_OF_RANGE': cls.__constants['STATUS_OUT_OF_RANGE'],
            'STATUS_TIMEOUT': cls.__constants['STATUS_TIMEOUT'],
            'STATUS_CONFIG_ERROR': cls.__constants['STATUS_CONFIG_ERROR'],
            'STATUS_OVERHEAT': cls.__constants['STATUS_OVERHEAT'],
            'STATUS_OVERCURRENT': cls.__constants['STATUS_OVERCURRENT'],
        }

    @property
    def STATUS_OK(self):
        """Message constant 'STATUS_OK'."""
        return Metaclass_RobomasterMotorState.__constants['STATUS_OK']

    @property
    def STATUS_NOT_INITIALIZED(self):
        """Message constant 'STATUS_NOT_INITIALIZED'."""
        return Metaclass_RobomasterMotorState.__constants['STATUS_NOT_INITIALIZED']

    @property
    def STATUS_CAN_ERROR(self):
        """Message constant 'STATUS_CAN_ERROR'."""
        return Metaclass_RobomasterMotorState.__constants['STATUS_CAN_ERROR']

    @property
    def STATUS_OUT_OF_RANGE(self):
        """Message constant 'STATUS_OUT_OF_RANGE'."""
        return Metaclass_RobomasterMotorState.__constants['STATUS_OUT_OF_RANGE']

    @property
    def STATUS_TIMEOUT(self):
        """Message constant 'STATUS_TIMEOUT'."""
        return Metaclass_RobomasterMotorState.__constants['STATUS_TIMEOUT']

    @property
    def STATUS_CONFIG_ERROR(self):
        """Message constant 'STATUS_CONFIG_ERROR'."""
        return Metaclass_RobomasterMotorState.__constants['STATUS_CONFIG_ERROR']

    @property
    def STATUS_OVERHEAT(self):
        """Message constant 'STATUS_OVERHEAT'."""
        return Metaclass_RobomasterMotorState.__constants['STATUS_OVERHEAT']

    @property
    def STATUS_OVERCURRENT(self):
        """Message constant 'STATUS_OVERCURRENT'."""
        return Metaclass_RobomasterMotorState.__constants['STATUS_OVERCURRENT']


class RobomasterMotorState(metaclass=Metaclass_RobomasterMotorState):
    """
    Message class 'RobomasterMotorState'.

    Constants:
      STATUS_OK
      STATUS_NOT_INITIALIZED
      STATUS_CAN_ERROR
      STATUS_OUT_OF_RANGE
      STATUS_TIMEOUT
      STATUS_CONFIG_ERROR
      STATUS_OVERHEAT
      STATUS_OVERCURRENT
    """

    __slots__ = [
        '_header',
        '_motor_id',
        '_current_position_rad',
        '_current_velocity_rps',
        '_current_milliamps',
        '_temperature_celsius',
        '_target_position_rad',
        '_target_velocity_rps',
        '_target_current_ma',
        '_control_mode',
        '_enabled',
        '_status',
        '_last_command_time_ms',
        '_saturation_count',
        '_timeout_count',
        '_error_count',
        '_overheat_count',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'motor_id': 'uint8',
        'current_position_rad': 'float',
        'current_velocity_rps': 'float',
        'current_milliamps': 'int16',
        'temperature_celsius': 'uint8',
        'target_position_rad': 'float',
        'target_velocity_rps': 'float',
        'target_current_ma': 'int16',
        'control_mode': 'uint8',
        'enabled': 'boolean',
        'status': 'uint8',
        'last_command_time_ms': 'uint32',
        'saturation_count': 'uint32',
        'timeout_count': 'uint32',
        'error_count': 'uint32',
        'overheat_count': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.motor_id = kwargs.get('motor_id', int())
        self.current_position_rad = kwargs.get('current_position_rad', float())
        self.current_velocity_rps = kwargs.get('current_velocity_rps', float())
        self.current_milliamps = kwargs.get('current_milliamps', int())
        self.temperature_celsius = kwargs.get('temperature_celsius', int())
        self.target_position_rad = kwargs.get('target_position_rad', float())
        self.target_velocity_rps = kwargs.get('target_velocity_rps', float())
        self.target_current_ma = kwargs.get('target_current_ma', int())
        self.control_mode = kwargs.get('control_mode', int())
        self.enabled = kwargs.get('enabled', bool())
        self.status = kwargs.get('status', int())
        self.last_command_time_ms = kwargs.get('last_command_time_ms', int())
        self.saturation_count = kwargs.get('saturation_count', int())
        self.timeout_count = kwargs.get('timeout_count', int())
        self.error_count = kwargs.get('error_count', int())
        self.overheat_count = kwargs.get('overheat_count', int())

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
        if self.current_position_rad != other.current_position_rad:
            return False
        if self.current_velocity_rps != other.current_velocity_rps:
            return False
        if self.current_milliamps != other.current_milliamps:
            return False
        if self.temperature_celsius != other.temperature_celsius:
            return False
        if self.target_position_rad != other.target_position_rad:
            return False
        if self.target_velocity_rps != other.target_velocity_rps:
            return False
        if self.target_current_ma != other.target_current_ma:
            return False
        if self.control_mode != other.control_mode:
            return False
        if self.enabled != other.enabled:
            return False
        if self.status != other.status:
            return False
        if self.last_command_time_ms != other.last_command_time_ms:
            return False
        if self.saturation_count != other.saturation_count:
            return False
        if self.timeout_count != other.timeout_count:
            return False
        if self.error_count != other.error_count:
            return False
        if self.overheat_count != other.overheat_count:
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
    def current_position_rad(self):
        """Message field 'current_position_rad'."""
        return self._current_position_rad

    @current_position_rad.setter
    def current_position_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_position_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current_position_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current_position_rad = value

    @builtins.property
    def current_velocity_rps(self):
        """Message field 'current_velocity_rps'."""
        return self._current_velocity_rps

    @current_velocity_rps.setter
    def current_velocity_rps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_velocity_rps' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current_velocity_rps' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current_velocity_rps = value

    @builtins.property
    def current_milliamps(self):
        """Message field 'current_milliamps'."""
        return self._current_milliamps

    @current_milliamps.setter
    def current_milliamps(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_milliamps' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'current_milliamps' field must be an integer in [-32768, 32767]"
        self._current_milliamps = value

    @builtins.property
    def temperature_celsius(self):
        """Message field 'temperature_celsius'."""
        return self._temperature_celsius

    @temperature_celsius.setter
    def temperature_celsius(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'temperature_celsius' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'temperature_celsius' field must be an unsigned integer in [0, 255]"
        self._temperature_celsius = value

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
    def last_command_time_ms(self):
        """Message field 'last_command_time_ms'."""
        return self._last_command_time_ms

    @last_command_time_ms.setter
    def last_command_time_ms(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'last_command_time_ms' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'last_command_time_ms' field must be an unsigned integer in [0, 4294967295]"
        self._last_command_time_ms = value

    @builtins.property
    def saturation_count(self):
        """Message field 'saturation_count'."""
        return self._saturation_count

    @saturation_count.setter
    def saturation_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'saturation_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'saturation_count' field must be an unsigned integer in [0, 4294967295]"
        self._saturation_count = value

    @builtins.property
    def timeout_count(self):
        """Message field 'timeout_count'."""
        return self._timeout_count

    @timeout_count.setter
    def timeout_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timeout_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'timeout_count' field must be an unsigned integer in [0, 4294967295]"
        self._timeout_count = value

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

    @builtins.property
    def overheat_count(self):
        """Message field 'overheat_count'."""
        return self._overheat_count

    @overheat_count.setter
    def overheat_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'overheat_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'overheat_count' field must be an unsigned integer in [0, 4294967295]"
        self._overheat_count = value
