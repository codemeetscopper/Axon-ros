#!/usr/bin/env python3
"""Minimal TCP echo server for sanity checks."""
import socket


def main() -> None:
    host = "0.0.0.0"
    port = 9000
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind((host, port))
        sock.listen(1)
        conn, _ = sock.accept()
        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                conn.sendall(data)


if __name__ == "__main__":
    main()
