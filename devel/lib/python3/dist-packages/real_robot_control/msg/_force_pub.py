# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from real_robot_control/force_pub.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class force_pub(genpy.Message):
  _md5sum = "01bd0bd3e5758946edad85592cef21e2"
  _type = "real_robot_control/force_pub"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64 X
float64 Y
float64 Z
float64 MX
float64 MY
float64 MZ"""
  __slots__ = ['X','Y','Z','MX','MY','MZ']
  _slot_types = ['float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       X,Y,Z,MX,MY,MZ

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(force_pub, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.X is None:
        self.X = 0.
      if self.Y is None:
        self.Y = 0.
      if self.Z is None:
        self.Z = 0.
      if self.MX is None:
        self.MX = 0.
      if self.MY is None:
        self.MY = 0.
      if self.MZ is None:
        self.MZ = 0.
    else:
      self.X = 0.
      self.Y = 0.
      self.Z = 0.
      self.MX = 0.
      self.MY = 0.
      self.MZ = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_6d().pack(_x.X, _x.Y, _x.Z, _x.MX, _x.MY, _x.MZ))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 48
      (_x.X, _x.Y, _x.Z, _x.MX, _x.MY, _x.MZ,) = _get_struct_6d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_6d().pack(_x.X, _x.Y, _x.Z, _x.MX, _x.MY, _x.MZ))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 48
      (_x.X, _x.Y, _x.Z, _x.MX, _x.MY, _x.MZ,) = _get_struct_6d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6d = None
def _get_struct_6d():
    global _struct_6d
    if _struct_6d is None:
        _struct_6d = struct.Struct("<6d")
    return _struct_6d