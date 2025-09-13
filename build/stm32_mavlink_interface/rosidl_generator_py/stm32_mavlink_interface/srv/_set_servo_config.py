# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stm32_mavlink_interface:srv/SetServoConfig.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetServoConfig_Request(type):
    """Metaclass of message 'SetServoConfig_Request'."""

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
                'stm32_mavlink_interface.srv.SetServoConfig_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_servo_config__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_servo_config__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_servo_config__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_servo_config__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_servo_config__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetServoConfig_Request(metaclass=Metaclass_SetServoConfig_Request):
    """Message class 'SetServoConfig_Request'."""

    __slots__ = [
        '_servo_id',
        '_angle_min_deg',
        '_angle_max_deg',
        '_pulse_min_us',
        '_pulse_max_us',
        '_pulse_neutral_us',
        '_direction_inverted',
        '_offset_deg',
        '_max_velocity_deg_per_s',
        '_max_acceleration_deg_per_s2',
        '_watchdog_timeout_ms',
        '_fail_safe_behavior',
        '_save_to_flash',
    ]

    _fields_and_field_types = {
        'servo_id': 'uint8',
        'angle_min_deg': 'float',
        'angle_max_deg': 'float',
        'pulse_min_us': 'uint16',
        'pulse_max_us': 'uint16',
        'pulse_neutral_us': 'uint16',
        'direction_inverted': 'boolean',
        'offset_deg': 'float',
        'max_velocity_deg_per_s': 'float',
        'max_acceleration_deg_per_s2': 'float',
        'watchdog_timeout_ms': 'uint32',
        'fail_safe_behavior': 'uint8',
        'save_to_flash': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.servo_id = kwargs.get('servo_id', int())
        self.angle_min_deg = kwargs.get('angle_min_deg', float())
        self.angle_max_deg = kwargs.get('angle_max_deg', float())
        self.pulse_min_us = kwargs.get('pulse_min_us', int())
        self.pulse_max_us = kwargs.get('pulse_max_us', int())
        self.pulse_neutral_us = kwargs.get('pulse_neutral_us', int())
        self.direction_inverted = kwargs.get('direction_inverted', bool())
        self.offset_deg = kwargs.get('offset_deg', float())
        self.max_velocity_deg_per_s = kwargs.get('max_velocity_deg_per_s', float())
        self.max_acceleration_deg_per_s2 = kwargs.get('max_acceleration_deg_per_s2', float())
        self.watchdog_timeout_ms = kwargs.get('watchdog_timeout_ms', int())
        self.fail_safe_behavior = kwargs.get('fail_safe_behavior', int())
        self.save_to_flash = kwargs.get('save_to_flash', bool())

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
        if self.servo_id != other.servo_id:
            return False
        if self.angle_min_deg != other.angle_min_deg:
            return False
        if self.angle_max_deg != other.angle_max_deg:
            return False
        if self.pulse_min_us != other.pulse_min_us:
            return False
        if self.pulse_max_us != other.pulse_max_us:
            return False
        if self.pulse_neutral_us != other.pulse_neutral_us:
            return False
        if self.direction_inverted != other.direction_inverted:
            return False
        if self.offset_deg != other.offset_deg:
            return False
        if self.max_velocity_deg_per_s != other.max_velocity_deg_per_s:
            return False
        if self.max_acceleration_deg_per_s2 != other.max_acceleration_deg_per_s2:
            return False
        if self.watchdog_timeout_ms != other.watchdog_timeout_ms:
            return False
        if self.fail_safe_behavior != other.fail_safe_behavior:
            return False
        if self.save_to_flash != other.save_to_flash:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
    def angle_min_deg(self):
        """Message field 'angle_min_deg'."""
        return self._angle_min_deg

    @angle_min_deg.setter
    def angle_min_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angle_min_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angle_min_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angle_min_deg = value

    @builtins.property
    def angle_max_deg(self):
        """Message field 'angle_max_deg'."""
        return self._angle_max_deg

    @angle_max_deg.setter
    def angle_max_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angle_max_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angle_max_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angle_max_deg = value

    @builtins.property
    def pulse_min_us(self):
        """Message field 'pulse_min_us'."""
        return self._pulse_min_us

    @pulse_min_us.setter
    def pulse_min_us(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pulse_min_us' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'pulse_min_us' field must be an unsigned integer in [0, 65535]"
        self._pulse_min_us = value

    @builtins.property
    def pulse_max_us(self):
        """Message field 'pulse_max_us'."""
        return self._pulse_max_us

    @pulse_max_us.setter
    def pulse_max_us(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pulse_max_us' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'pulse_max_us' field must be an unsigned integer in [0, 65535]"
        self._pulse_max_us = value

    @builtins.property
    def pulse_neutral_us(self):
        """Message field 'pulse_neutral_us'."""
        return self._pulse_neutral_us

    @pulse_neutral_us.setter
    def pulse_neutral_us(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pulse_neutral_us' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'pulse_neutral_us' field must be an unsigned integer in [0, 65535]"
        self._pulse_neutral_us = value

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
    def offset_deg(self):
        """Message field 'offset_deg'."""
        return self._offset_deg

    @offset_deg.setter
    def offset_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'offset_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'offset_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._offset_deg = value

    @builtins.property
    def max_velocity_deg_per_s(self):
        """Message field 'max_velocity_deg_per_s'."""
        return self._max_velocity_deg_per_s

    @max_velocity_deg_per_s.setter
    def max_velocity_deg_per_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_velocity_deg_per_s' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'max_velocity_deg_per_s' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._max_velocity_deg_per_s = value

    @builtins.property
    def max_acceleration_deg_per_s2(self):
        """Message field 'max_acceleration_deg_per_s2'."""
        return self._max_acceleration_deg_per_s2

    @max_acceleration_deg_per_s2.setter
    def max_acceleration_deg_per_s2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_acceleration_deg_per_s2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'max_acceleration_deg_per_s2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._max_acceleration_deg_per_s2 = value

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
    def fail_safe_behavior(self):
        """Message field 'fail_safe_behavior'."""
        return self._fail_safe_behavior

    @fail_safe_behavior.setter
    def fail_safe_behavior(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fail_safe_behavior' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'fail_safe_behavior' field must be an unsigned integer in [0, 255]"
        self._fail_safe_behavior = value

    @builtins.property
    def save_to_flash(self):
        """Message field 'save_to_flash'."""
        return self._save_to_flash

    @save_to_flash.setter
    def save_to_flash(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'save_to_flash' field must be of type 'bool'"
        self._save_to_flash = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetServoConfig_Response(type):
    """Metaclass of message 'SetServoConfig_Response'."""

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
                'stm32_mavlink_interface.srv.SetServoConfig_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_servo_config__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_servo_config__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_servo_config__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_servo_config__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_servo_config__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetServoConfig_Response(metaclass=Metaclass_SetServoConfig_Response):
    """Message class 'SetServoConfig_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_SetServoConfig(type):
    """Metaclass of service 'SetServoConfig'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stm32_mavlink_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stm32_mavlink_interface.srv.SetServoConfig')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_servo_config

            from stm32_mavlink_interface.srv import _set_servo_config
            if _set_servo_config.Metaclass_SetServoConfig_Request._TYPE_SUPPORT is None:
                _set_servo_config.Metaclass_SetServoConfig_Request.__import_type_support__()
            if _set_servo_config.Metaclass_SetServoConfig_Response._TYPE_SUPPORT is None:
                _set_servo_config.Metaclass_SetServoConfig_Response.__import_type_support__()


class SetServoConfig(metaclass=Metaclass_SetServoConfig):
    from stm32_mavlink_interface.srv._set_servo_config import SetServoConfig_Request as Request
    from stm32_mavlink_interface.srv._set_servo_config import SetServoConfig_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
