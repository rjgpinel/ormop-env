import struct
import ujson

import numpy as np
from ormope.protocol.base import Message, MessageType


class ResetRequest(Message):

    def __init__(self):
        super(ResetRequest, self).__init__(MessageType.RESET_REQUEST)

class ResetResponse(Message):

    def __init__(self, observation):
        super(ResetResponse, self).__init__(MessageType.RESET_RESPONSE)
        self.observation = observation

    def _serialize(self):
        payload = ujson.dumps(self.observation).encode("utf-8")
        return payload

    @classmethod
    def deserialize(cls, msg_bytes):
        observation = ujson.loads(msg_bytes.decode("utf-8"))
        return cls(observation)
