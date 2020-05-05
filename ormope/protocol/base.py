import struct

from enum import Enum


class MessageType(Enum):
    MAKE_REQUEST = 0
    MAKE_RESPONSE = 1
    RESET_REQUEST = 2
    RESET_RESPONSE = 3
    STEP_REQUEST = 4
    STEP_RESPONSE = 5
    CLOSE_REQUEST = 6
    CLOSE_RESPONSE = 7


class MessageHeader(object):
    # Header size is fixed to 2 (msg_type) + 4 (payload_size) bytes
    SIZE = 6

    def __init__(self, msg_type, payload_size):
        self.msg_type = msg_type
        self.payload_size = payload_size

    def serialize(self):
        header_bytes = struct.pack('!HI', self.msg_type.value, self.payload_size)
        return header_bytes

    @classmethod
    def deserialize(cls, msg_bytes):
        msg_type, payload_size = struct.unpack('!HI', msg_bytes[:MessageHeader.SIZE])
        return cls(MessageType(msg_type), payload_size)

class Message(object):

    def __init__(self, msg_type):
        self.msg_type = msg_type

    def _serialize(self):
        return b''

    def serialize(self):
        payload = self._serialize()
        msg_bytes = MessageHeader(self.msg_type, len(payload)).serialize() + payload
        return msg_bytes

    @classmethod
    def deserialize(cls, msg_bytes):
        return cls()
