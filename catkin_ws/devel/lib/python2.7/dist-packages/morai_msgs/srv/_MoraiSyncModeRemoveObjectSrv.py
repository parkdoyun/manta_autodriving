# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from morai_msgs/MoraiSyncModeRemoveObjectSrvRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import morai_msgs.msg

class MoraiSyncModeRemoveObjectSrvRequest(genpy.Message):
  _md5sum = "f29d91eaba9c9b9e22406f170d281a6c"
  _type = "morai_msgs/MoraiSyncModeRemoveObjectSrvRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """SyncModeRemoveObject request

================================================================================
MSG: morai_msgs/SyncModeRemoveObject
int32 unique_id
uint64 frame
"""
  __slots__ = ['request']
  _slot_types = ['morai_msgs/SyncModeRemoveObject']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       request

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MoraiSyncModeRemoveObjectSrvRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.request is None:
        self.request = morai_msgs.msg.SyncModeRemoveObject()
    else:
      self.request = morai_msgs.msg.SyncModeRemoveObject()

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
      buff.write(_get_struct_iQ().pack(_x.request.unique_id, _x.request.frame))
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
      if self.request is None:
        self.request = morai_msgs.msg.SyncModeRemoveObject()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.request.unique_id, _x.request.frame,) = _get_struct_iQ().unpack(str[start:end])
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
      buff.write(_get_struct_iQ().pack(_x.request.unique_id, _x.request.frame))
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
      if self.request is None:
        self.request = morai_msgs.msg.SyncModeRemoveObject()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.request.unique_id, _x.request.frame,) = _get_struct_iQ().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_iQ = None
def _get_struct_iQ():
    global _struct_iQ
    if _struct_iQ is None:
        _struct_iQ = struct.Struct("<iQ")
    return _struct_iQ
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from morai_msgs/MoraiSyncModeRemoveObjectSrvResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import morai_msgs.msg

class MoraiSyncModeRemoveObjectSrvResponse(genpy.Message):
  _md5sum = "4039c80fa74cc3be5f583706bf97e6b0"
  _type = "morai_msgs/MoraiSyncModeRemoveObjectSrvResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """SyncModeResultResponse response


================================================================================
MSG: morai_msgs/SyncModeResultResponse
bool result
"""
  __slots__ = ['response']
  _slot_types = ['morai_msgs/SyncModeResultResponse']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       response

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MoraiSyncModeRemoveObjectSrvResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.response is None:
        self.response = morai_msgs.msg.SyncModeResultResponse()
    else:
      self.response = morai_msgs.msg.SyncModeResultResponse()

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
      _x = self.response.result
      buff.write(_get_struct_B().pack(_x))
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
      if self.response is None:
        self.response = morai_msgs.msg.SyncModeResultResponse()
      end = 0
      start = end
      end += 1
      (self.response.result,) = _get_struct_B().unpack(str[start:end])
      self.response.result = bool(self.response.result)
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
      _x = self.response.result
      buff.write(_get_struct_B().pack(_x))
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
      if self.response is None:
        self.response = morai_msgs.msg.SyncModeResultResponse()
      end = 0
      start = end
      end += 1
      (self.response.result,) = _get_struct_B().unpack(str[start:end])
      self.response.result = bool(self.response.result)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class MoraiSyncModeRemoveObjectSrv(object):
  _type          = 'morai_msgs/MoraiSyncModeRemoveObjectSrv'
  _md5sum = 'd499baec7083a3731b7e7fc183807dce'
  _request_class  = MoraiSyncModeRemoveObjectSrvRequest
  _response_class = MoraiSyncModeRemoveObjectSrvResponse
