"""Atlas-discovered existing navigation capabilities; no ROS command publisher."""

import math


class NavigationClient:
    PREFIX = 'robonix/service/navigation/navigate'

    def __init__(self, provider_id='nav2', consumer_id='outdoor_navigation'):
        import grpc
        from robonix_api import ATLAS
        from robonix_api.atlas_types import LifecycleState
        import navigation_pb2
        import robonix_contracts_pb2_grpc
        self.grpc, self.atlas = grpc, ATLAS
        self.active_state = LifecycleState.ACTIVE
        self.messages, self.stubs = navigation_pb2, robonix_contracts_pb2_grpc
        self.provider_id, self.consumer_id = provider_id, consumer_id
        self.connections = {}
        self.registration_id = None

    def inspect(self):
        # The current SDK Query facade has no timeout argument. Bound the wire
        # RPC so a stalled discovery plane cannot stall cancellation forever.
        from robonix_api.atlas_types import from_pb_provider
        pb = self.atlas._wire_pb
        response = self.atlas._wire_stub.Query(pb.QueryRequest(kind=pb.KIND_SERVICE,
            id=self.provider_id, contract_id=self.PREFIX, transport=pb.TRANSPORT_GRPC), timeout=2)
        providers = [from_pb_provider(p) for p in response.providers]
        if len(providers) != 1:
            return {'active': False, 'registration_id': None}
        provider = providers[0]
        if provider.registration_id != self.registration_id:
            self.close()
            self.registration_id = provider.registration_id
        return {'active': provider.state == self.active_state, 'registration_id': provider.registration_id}

    def _stub(self, suffix, name):
        contract = self.PREFIX + suffix
        if contract not in self.connections:
            edge = self.atlas.connect_capability(consumer_id=self.consumer_id,
                provider_id=self.provider_id, contract_id=contract, transport='grpc')
            channel = self.grpc.insecure_channel(edge.endpoint, options=[('grpc.enable_http_proxy', 0)])
            self.connections[contract] = (edge, channel, getattr(self.stubs, name)(channel))
        return self.connections[contract][2]

    def goal_request(self, goal):
        request = self.messages.Navigate_Request()
        request.goal.header.frame_id = 'map'
        request.goal.pose.position.x = goal['x']
        request.goal.pose.position.y = goal['y']
        request.goal.pose.orientation.z = math.sin(goal['yaw'] / 2)
        request.goal.pose.orientation.w = math.cos(goal['yaw'] / 2)
        return request

    def navigate(self, goal):
        response = self._stub('', 'RobonixServiceNavigationNavigateStub').Navigate(self.goal_request(goal), timeout=3)
        return {'accepted': response.accepted, 'run_id': response.run_id, 'detail': response.detail}

    def status(self, run_id):
        if not run_id:
            raise ValueError('Explicit owned run_id is required')
        response = self._stub('/status', 'RobonixServiceNavigationNavigateStatusStub').GetNavigationStatus(
            self.messages.GetNavigationStatus_Request(run_id=run_id), timeout=3)
        return {'known': response.known, 'state': response.state, 'detail': response.detail}

    def cancel(self, run_id):
        if not run_id:
            raise ValueError('Never cancel another client’s current goal')
        response = self._stub('/cancel', 'RobonixServiceNavigationNavigateCancelStub').CancelNavigation(
            self.messages.CancelNavigation_Request(run_id=run_id), timeout=3)
        return {'accepted': response.accepted, 'detail': response.detail}

    def close(self):
        for edge, channel, _ in self.connections.values():
            channel.close()
            edge.close()
        self.connections.clear()
