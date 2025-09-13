# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stm32_mavlink_interface:srv/SetEncoderConfig.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetEncoderConfig_Request(type):
    """Metaclass of message 'SetEncoderConfig_Request'."""

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
                'stm32_mavlink_interface.srv.SetEncoderConfig_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_encoder_config__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_encoder_config__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_encoder_config__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_encoder_config__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_encoder_config__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetEncoderConfig_Request(metaclass=Metaclass_SetEncoderConfig_Request):
    """Message class 'SetEncoderConfig_Request'."""

    __slots__ = [
        '_encoder_id',
        '_cpr',
        '_invert_a',
        '_invert_b',
        '_use_z',
        '_watchdog_timeout_ms',
        '_offset_counts',
        '_wrap_around',
        '_save_to_flash',
    ]

    _fields_and_field_types = {
        'encoder_id': 'uint8',
        'cpr': 'uint16',
        'invert_a': 'boolean',
        'invert_b': 'boolean',
        'use_z': 'boolean',
        'watchdog_timeout_ms': 'uint32',
        'offset_counts': 'int32',
        'wrap_around': 'boolean',
        'save_to_flash': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.encoder_id = kwargs.get('encoder_id', int())
        self.cpr = kwargs.get('cpr', int())
        self.invert_a = kwargs.get('invert_a', bool())
        self.invert_b = kwargs.get('invert_b', bool())
        self.use_z = kwargs.get('use_z', bool())
        self.watchdog_timeout_ms = kwargs.get('watchdog_timeout_ms', int())
        self.offset_counts = kwargs.get('offset_counts', int())
        self.wrap_around = kwargs.get('wrap_around', bool())
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
        if self.encoder_id != other.encoder_id:
            return False
        if self.cpr != other.cpr:
            return False
        if self.invert_a != other.invert_a:
            return False
        if self.invert_b != other.invert_b:
            return False
        if self.use_z != other.use_z:
            return False
        if self.watchdog_timeout_ms != other.watchdog_timeout_ms:
            return False
        if self.offset_counts != other.offset_counts:
            return False
        if self.wrap_around != other.wrap_around:
            return False
        if self.save_to_flash != other.save_to_flash:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
    def cpr(self):
        """Message field 'cpr'."""
        return self._cpr

    @cpr.setter
    def cpr(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cpr' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'cpr' field must be an unsigned integer in [0, 65535]"
        self._cpr = value

    @builtins.property
    def invert_a(self):
        """Message field 'invert_a'."""
        return self._invert_a

    @invert_a.setter
    def invert_a(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'invert_a' field must be of type 'bool'"
        self._invert_a = value

    @builtins.property
    def invert_b(self):
        """Message field 'invert_b'."""
        return self._invert_b

    @invert_b.setter
    def invert_b(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'invert_b' field must be of type 'bool'"
        self._invert_b = value

    @builtins.property
    def use_z(self):
        """Message field 'use_z'."""
        return self._use_z

    @use_z.setter
    def use_z(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_z' field must be of type 'bool'"
        self._use_z = value

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
    def offset_counts(self):
        """Message field 'offset_counts'."""
        return self._offset_counts

    @offset_counts.setter
    def offset_counts(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'offset_counts' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'offset_counts' field must be an integer in [-2147483648, 2147483647]"
        self._offset_counts = value

    @builtins.property
    def wrap_around(self):
        """Message field 'wrap_around'."""
        return self._wrap_around

    @wrap_around.setter
    def wrap_around(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'wrap_around' field must be of type 'bool'"
        self._wrap_around = value

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


class Metaclass_SetEncoderConfig_Response(type):
    """Metaclass of message 'SetEncoderConfig_Response'."""

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
                'stm32_mavlink_interface.srv.SetEncoderConfig_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_encoder_config__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_encoder_config__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_encoder_config__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_encoder_config__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_encoder_config__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetEncoderConfig_Response(metaclass=Metaclass_SetEncoderConfig_Response):
    """Message class 'SetEncoderConfig_Response'."""

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


class Metaclass_SetEncoderConfig(type):
    """Metaclass of service 'SetEncoderConfig'."""

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
                'stm32_mavlink_interface.srv.SetEncoderConfig')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_encoder_config

            from stm32_mavlink_interface.srv import _set_encoder_config
            if _set_encoder_config.Metaclass_SetEncoderConfig_Request._TYPE_SUPPORT is None:
                _set_encoder_config.Metaclass_SetEncoderConfig_Request.__import_type_support__()
            if _set_encoder_config.Metaclass_SetEncoderConfig_Response._TYPE_SUPPORT is None:
                _set_encoder_config.Metaclass_SetEncoderConfig_Response.__import_type_support__()


class SetEncoderConfig(metaclass=Metaclass_SetEncoderConfig):
    from stm32_mavlink_interface.srv._set_encoder_config import SetEncoderConfig_Request as Request
    from stm32_mavlink_interface.srv._set_encoder_config import SetEncoderConfig_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
