import struct

from ormope.protocol.base import Message, MessageType


class MakeRequest(Message):

    def __init__(self, env_name):
        super(MakeRequest, self).__init__(MessageType.MAKE_REQUEST)
        self.env_name = env_name

    def _serialize(self):
        payload = self.env_name.encode("utf-8")
        return payload

    @classmethod
    def deserialize(cls, msg_bytes):
        env_name = msg_bytes.decode("utf-8")
        return cls(env_name)


class MakeResponse(Message):

    def __init__(self, result):
        super(MakeResponse, self).__init__(MessageType.MAKE_RESPONSE)
        self.result = result

    def _serialize(self):
        payload = struct.pack('!?', self.result)
        return payload

    @classmethod
    def deserialize(cls, msg_bytes):
        result = struct.unpack_from('!?', msg_bytes)
        return cls(result)

