"""Send one map-frame goal through the existing navigation capability."""

import argparse
import math
import sys
import time
import uuid

import grpc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--provider', default='nav2')
    parser.add_argument('--wait', type=float, default=90.0)
    args = parser.parse_args()
    from robonix_api import ATLAS
    consumer = 'outdoor_testdrive_' + uuid.uuid4().hex[:8]
    ATLAS.register_service(consumer, 'robonix/service/outdoor_testdrive')
    try:
        edge = ATLAS.connect_capability(consumer_id=consumer, provider_id=args.provider,
            contract_id='robonix/service/navigation/navigate', transport='grpc')
        channel = grpc.insecure_channel(edge.endpoint, options=[('grpc.enable_http_proxy', 0)])
        import navigation_pb2, robonix_contracts_pb2_grpc
        stub = robonix_contracts_pb2_grpc.RobonixServiceNavigationNavigateStub(channel)
        req = navigation_pb2.Navigate_Request()
        req.goal.header.frame_id = 'map'
        req.goal.pose.position.x = args.x
        req.goal.pose.position.y = args.y
        req.goal.pose.orientation.z = math.sin(args.yaw / 2)
        req.goal.pose.orientation.w = math.cos(args.yaw / 2)
        out = stub.Navigate(req, timeout=5)
        print('navigate:', out.accepted, out.run_id, out.detail, flush=True)
        if not out.accepted:
            return 1
        edge2 = ATLAS.connect_capability(consumer_id=consumer, provider_id=args.provider,
            contract_id='robonix/service/navigation/navigate/status', transport='grpc')
        ch2 = grpc.insecure_channel(edge2.endpoint, options=[('grpc.enable_http_proxy', 0)])
        status = robonix_contracts_pb2_grpc.RobonixServiceNavigationNavigateStatusStub(ch2)
        deadline = time.time() + args.wait
        while time.time() < deadline:
            time.sleep(2)
            st = status.GetNavigationStatus(navigation_pb2.GetNavigationStatus_Request(run_id=out.run_id), timeout=5)
            print('status:', st.state, st.detail, flush=True)
            if st.state in ('SUCCEEDED', 'FAILED', 'CANCELED'):
                return 0 if st.state == 'SUCCEEDED' else 1
        return 2
    finally:
        ATLAS.unregister(consumer)


if __name__ == '__main__':
    sys.exit(main())
