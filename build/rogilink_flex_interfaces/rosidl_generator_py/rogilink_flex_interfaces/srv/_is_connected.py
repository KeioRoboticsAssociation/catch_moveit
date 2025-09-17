# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rogilink_flex_interfaces:srv/IsConnected.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IsConnected_Request(type):
    """Metaclass of message 'IsConnected_Request'."""

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
            module = import_type_support('rogilink_flex_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rogilink_flex_interfaces.srv.IsConnected_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__is_connected__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__is_connected__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__is_connected__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__is_connected__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__is_connected__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IsConnected_Request(metaclass=Metaclass_IsConnected_Request):
    """Message class 'IsConnected_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_IsConnected_Response(type):
    """Metaclass of message 'IsConnected_Response'."""

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
            module = import_type_support('rogilink_flex_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rogilink_flex_interfaces.srv.IsConnected_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__is_connected__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__is_connected__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__is_connected__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__is_connected__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__is_connected__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IsConnected_Response(metaclass=Metaclass_IsConnected_Response):
    """Message class 'IsConnected_Response'."""

    __slots__ = [
        '_connected',
        '_device_id',
    ]

    _fields_and_field_types = {
        'connected': 'boolean',
        'device_id': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.connected = kwargs.get('connected', bool())
        self.device_id = kwargs.get('device_id', int())

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
        if self.connected != other.connected:
            return False
        if self.device_id != other.device_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def connected(self):
        """Message field 'connected'."""
        return self._connected

    @connected.setter
    def connected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'connected' field must be of type 'bool'"
        self._connected = value

    @builtins.property
    def device_id(self):
        """Message field 'device_id'."""
        return self._device_id

    @device_id.setter
    def device_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'device_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'device_id' field must be an unsigned integer in [0, 255]"
        self._device_id = value


class Metaclass_IsConnected(type):
    """Metaclass of service 'IsConnected'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rogilink_flex_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rogilink_flex_interfaces.srv.IsConnected')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__is_connected

            from rogilink_flex_interfaces.srv import _is_connected
            if _is_connected.Metaclass_IsConnected_Request._TYPE_SUPPORT is None:
                _is_connected.Metaclass_IsConnected_Request.__import_type_support__()
            if _is_connected.Metaclass_IsConnected_Response._TYPE_SUPPORT is None:
                _is_connected.Metaclass_IsConnected_Response.__import_type_support__()


class IsConnected(metaclass=Metaclass_IsConnected):
    from rogilink_flex_interfaces.srv._is_connected import IsConnected_Request as Request
    from rogilink_flex_interfaces.srv._is_connected import IsConnected_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
