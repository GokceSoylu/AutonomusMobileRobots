import time
import zmq


class RemoteAPIClient:
    def __init__(self, program_name='b0RemoteApi', server_name='b0RemoteApi', timeout=5000, ros_compatible=False):
        self.program_name = program_name
        self.server_name = server_name
        self.ros_compatible = ros_compatible
        self.timeout = timeout
        self.ctx = zmq.Context()
        self.req = self.ctx.socket(zmq.REQ)
        self.req.connect("tcp://localhost:23000")
        self._id = 0

    def call(self, func, *args):
        self._id += 1
        msg = [self.program_name.encode(), str(self._id).encode(), func.encode()] + [str(a).encode() for a in args]
        self.req.send_multipart(msg)
        reply = self.req.recv_multipart()
        return reply

    def sim(self):
        return RemoteObject('sim', self)


class RemoteObject:
    def __init__(self, name, client):
        self._name = name
        self._client = client

    def __getattr__(self, name):
        def f(*args):
            return self._client.call(self._name + '.' + name, *args)
        return f
