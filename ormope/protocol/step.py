import ujson
import struct

from ormope.protocol.base import Message, MessageType


class StepRequest(Message):

    def __init__(self, action):
        super(StepRequest, self).__init__(MessageType.STEP_REQUEST)
        self.action = action

    def _serialize(self):
        payload = ujson.dumps(self.action).encode("utf-8")
        return payload

    @classmethod
    def deserialize(cls, msg_bytes):
        action = ujson.loads(msg_bytes.decode("utf-8"))
        return cls(action)


class StepResponse(Message):

    def __init__(self, observation, reward, done, info):
        super(StepResponse, self).__init__(MessageType.STEP_RESPONSE)
        self.observation = observation
        self.reward = reward
        self.done = done
        self.info = info

    def _serialize(self):
        observation_bytes = ujson.dumps(self.observation).encode("utf-8")
        payload = struct.pack("!I", len(observation_bytes)) + observation_bytes
        payload += struct.pack("!f", self.reward)
        payload += struct.pack("!?", self.done)
        info_bytes = ujson.dumps(self.info).encode("utf-8")
        payload += struct.pack("!I", len(info_bytes)) + info_bytes
        return payload

    @classmethod
    def deserialize(cls, msg_bytes):
        observation_size_bytes = 4
        observation_size = struct.unpack('!I', msg_bytes[:observation_size_bytes])[0]
        end_observation = observation_size_bytes + observation_size
        obsevation = ujson.loads(msg_bytes[observation_size_bytes:end_observation].decode("utf-8"))

        end_reward = end_observation + 4
        reward = struct.unpack('!f', msg_bytes[end_observation:end_reward])[0]

        end_done = end_reward + 1
        done = struct.unpack('!?', msg_bytes[end_reward:end_done])

        end_info_size = end_done + 4
        info_size = struct.unpack('!I', msg_bytes[end_done:end_info_size])[0]

        end_info = end_info_size + info_size
        info = ujson.loads(msg_bytes[end_info_size:end_info].decode("utf-8"))

        return cls(obsevation, reward, done, info)
