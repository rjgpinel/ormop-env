import time
import click
import socket

# import gtimer as gt

from ormope.envs.base_env import BaseEnv
from ormope.protocol.base import MessageType, MessageHeader
from ormope.protocol.make import MakeRequest, MakeResponse
from ormope.protocol.reset import ResetRequest, ResetResponse
from ormope.protocol.step import StepRequest, StepResponse
from ormope.protocol.close import CloseRequest, CloseResponse


class EnvironmentServer():

    def __init__(self, ):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def bind(self, host, port):
        self.sock.bind((host, port))
        self.sock.listen(1)

    def make(self, env_name):
        # print(env_name)
        # TODO: Create env using given name and gym
        self.env = BaseEnv()
        return True

    def receive(self, conn):
        # print('RECEIVING')
        data = conn.recv(MessageHeader.SIZE)
        header = MessageHeader.deserialize(data)
        # print("RECEIVING: %s" % str(header.msg_type))
        if header.payload_size > 0:
            payload_bytes = conn.recv(header.payload_size)
        else:
            payload_bytes = None

        return header, payload_bytes

    # @gt.wrap
    def send(self, conn, msg):
        # print("SENDING: %s" % str(msg.msg_type))
        msg_bytes = msg.serialize()
        # gt.stamp("Serialize data")
        conn.sendall(msg_bytes)
        # gt.stamp("Send data")

    # @gt.wrap
    def handle(self):
        print('WAITING FOR AN INCOME CONNECTION')
        while True:
            conn, addr = self.sock.accept()
            i = 0
            while True:
                header, payload_bytes = self.receive(conn)
                # gt.stamp("Receive Header %s : %i" % (header, i))
                if header.msg_type == MessageType.MAKE_REQUEST:
                    req = MakeRequest.deserialize(payload_bytes)
                    self.make(req.env_name)
                    # TODO: Handle exceptions when creating the environment
                    resp = MakeResponse(True)
                elif header.msg_type == MessageType.RESET_REQUEST:
                    req = ResetRequest.deserialize(payload_bytes)
                    observation = self.env.reset()
                    resp = ResetResponse(observation)
                elif header.msg_type == MessageType.STEP_REQUEST:
                    req = StepRequest.deserialize(payload_bytes)
                    # gt.stamp("Deserialize %i" %i)
                    observation, reward, done, info = self.env.step(req.action)
                    # gt.stamp("Take Step %i" % i)
                    resp = StepResponse(observation, reward, done, info)
                elif header.msg_type == MessageType.CLOSE_REQUEST:
                    req = CloseRequest.deserialize(payload_bytes)
                    self.env.close()
                    resp = CloseResponse(True)
                else:
                    print("Unknown message. Ignoring...")
                i+=1
                self.send(conn, resp)

                if resp.msg_type == MessageType.CLOSE_RESPONSE:
                    break


@click.command()
@click.option("-h", "--host", default="localhost")
@click.option("-p", "--port", default=20205)
def run(host, port):
    env = EnvironmentServer()
    env.bind(host, port)
    env.handle()
    # print(gt.report())

if __name__ == "__main__":
    run()



