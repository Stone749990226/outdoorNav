"""Drive a bounded straight segment with the chassis primitive (calibration only)."""

import argparse
import sys
import uuid

import grpc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--forward-m', type=float, required=True)
    parser.add_argument('--provider', default='lynx_quadruped')
    args = parser.parse_args()
    if not 0 < abs(args.forward_m) <= 10:
        raise SystemExit('Refusing a segment outside 0..10 m')
    from robonix_api import ATLAS
    from robonix_api.atlas_types import from_pb_provider
    pb = ATLAS._wire_pb
    response = ATLAS._wire_stub.Query(pb.QueryRequest(kind=pb.KIND_PRIMITIVE, id=args.provider,
        contract_id='robonix/primitive/chassis/move', transport=pb.TRANSPORT_GRPC), timeout=3)
    providers = [from_pb_provider(p) for p in response.providers]
    if len(providers) != 1:
        raise SystemExit('Expected exactly one chassis move provider')
    consumer = 'outdoor_calibration_' + uuid.uuid4().hex[:8]
    ATLAS.register_service(consumer, 'robonix/service/outdoor_calibration')
    try:
        edge = ATLAS.connect_capability(consumer_id=consumer, provider_id=args.provider,
            contract_id='robonix/primitive/chassis/move', transport='grpc')
        channel = grpc.insecure_channel(edge.endpoint, options=[('grpc.enable_http_proxy', 0)])
        import chassis_pb2_grpc
        stub = chassis_pb2_grpc.RobonixPrimitiveChassisMoveStub(channel)
        import chassis_pb2
        request = chassis_pb2.ExecuteMoveCommand_Request()
        request.command.forward_m = float(args.forward_m)
        result = stub.ExecuteMoveCommand(request, timeout=120)
        print(result)
    finally:
        ATLAS.unregister(consumer)


if __name__ == '__main__':
    sys.exit(main())
