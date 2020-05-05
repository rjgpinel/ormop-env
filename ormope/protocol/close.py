import struct

from ormope.protocol.base import Message, MessageType


class CloseRequest(Message):

    def __init__(self):
        super(CloseRequest, self).__init__(MessageType.CLOSE_REQUEST)


class CloseResponse(Message):

    def __init__(self, closed):
        super(CloseResponse, self).__init__(MessageType.CLOSE_RESPONSE)
        self.closed = closed

    def _serialize(self):
        payload = struct.pack("!?", self.closed)
        return payload

    @classmethod
    def deserialize(cls, msg_bytes):
        closed = struct.unpack("!?", msg_bytes)[0]
        return cls(closed)
