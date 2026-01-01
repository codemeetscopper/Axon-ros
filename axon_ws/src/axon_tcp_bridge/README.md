# axon_tcp_bridge

Architecture placeholder for a future TCP bridge that will expose ROS topics/services to external clients.

## Design Goals
- Session management with keepalive policies.
- Topic and service bridging with explicit schemas.
- Minimal, testable core types before full networking.

## Contents
- `axon_tcp_bridge.bridge`: skeleton types (`ClientSession`, `SessionManager`, `KeepalivePolicy`, `TopicBridge`).
- `scripts/echo_server.py`: minimal TCP echo server for sanity checks.
