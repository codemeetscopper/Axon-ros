"""Architecture placeholders for TCP bridge."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class KeepalivePolicy:
    interval_s: float = 5.0
    timeout_s: float = 15.0


@dataclass
class ClientSession:
    session_id: str
    connected: bool = True
    last_seen_s: float = 0.0


@dataclass
class SessionManager:
    sessions: Dict[str, ClientSession] = field(default_factory=dict)

    def register(self, session: ClientSession) -> None:
        self.sessions[session.session_id] = session

    def remove(self, session_id: str) -> None:
        self.sessions.pop(session_id, None)

    def get(self, session_id: str) -> Optional[ClientSession]:
        return self.sessions.get(session_id)


@dataclass
class TopicBridge:
    """Placeholder for future topic bridging logic."""
    topic_name: str
    qos_profile: str = "default"
