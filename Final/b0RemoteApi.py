from b0 import RemoteAPIClient

class RemoteApiClient:
    def __init__(self, scene='b0RemoteApi'):
        self._client = RemoteAPIClient('b0RemoteApi', scene)
        self.sim = self._client.sim
