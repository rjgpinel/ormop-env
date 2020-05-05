import click
import time
import socket
import numpy as np

# import gtimer as gt
import time

from ormope.protocol.base import MessageHeader, MessageType
from ormope.protocol.make import MakeRequest, MakeResponse
from ormope.protocol.reset import ResetRequest, ResetResponse
from ormope.protocol.step import StepRequest, StepResponse
from ormope.protocol.close import CloseRequest, CloseResponse


class EnvironmentClient:
    """demonstration class only
      - coded for clarity, not efficiency
    """

    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(
                            socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

    def connect(self, host, port):
        self.sock.connect((host, port))

    def send(self, msg):
        # print("SENDING: %s" % str(msg.msg_type))
        self.sock.sendall(msg.serialize())

    # @gt.wrap
    def receive(self):
        # print("RECEIVING")
        data = self.sock.recv(MessageHeader.SIZE)
        # gt.stamp("Receive Header")
        header = MessageHeader.deserialize(data)
        # gt.stamp("Deserialize header")
        # print("RECEIVING: %s" % str(header.msg_type))
        # gt.stamp("Tcv message")
        if header.payload_size > 0:
            payload_bytes = self.sock.recv(header.payload_size)
        else:
            payload_bytes = None

        if header.msg_type == MessageType.MAKE_RESPONSE:
            resp = MakeResponse.deserialize(payload_bytes)
        elif header.msg_type == MessageType.RESET_RESPONSE:
            resp = ResetResponse.deserialize(payload_bytes)
        elif header.msg_type == MessageType.STEP_RESPONSE:
            resp = StepResponse.deserialize(payload_bytes)
        elif header.msg_type == MessageType.CLOSE_RESPONSE:
            resp = CloseResponse.deserialize(payload_bytes)
        else:
            resp = None
            print("Unknown MessageType. Ignoring message.")
        return resp

    def make(self, env_name):
        req = MakeRequest(env_name)
        self.send(req)
        resp = self.receive()
        return resp.result

    def reset(self):
        req = ResetRequest()
        self.send(req)
        resp = self.receive()
        return resp.observation

    # @gt.wrap
    def step(self, action):
        req = StepRequest(action)
        self.send(req)
        resp = self.receive()
        return resp.observation, resp.reward, resp.done, resp.info

    def close(self):
        req = CloseRequest()
        self.send(req)
        resp = self.receive()
        return resp.closed

@click.command()
@click.option("-h", "--host", default="localhost")
@click.option("-p", "--port", default=20205)
def run(host, port):
    env = EnvironmentClient()

    env.connect(host, port)
    # print(env.make("hello"))
    env.make("hello")
    all_times = []
    for i in range(100):
        env.reset()
        times = []
        for i in range(20):
            initial = time.time()
            result = env.step([0.0, 0.0, 0.0, 0.0, 0.0])
            # gt.stamp("Step %i" % i)
            end = time.time()
            times.append(end-initial)
        all_times.append(np.mean(times))
    print(np.mean(all_times))
    env.close()
    # print(gt.report())
if __name__ == "__main__":
    run()
