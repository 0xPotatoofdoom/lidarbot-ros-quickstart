#!/usr/bin/env python3
"""Relay micro-ROS XRCE traffic between the robot (UDP) and the agent (TCP).

Docker Desktop on macOS delivers inbound UDP to containers but drops the
replies, so the robot never hears the agent. This runs natively on the Mac:
it receives the robot's UDP datagrams on --udp-port, forwards each one to a
micro-ROS agent started with `tcp4 --port <tcp-port>` inside the container
(XRCE-over-TCP framing: 2-byte little-endian length prefix), and sends the
agent's framed replies back to the robot as UDP datagrams.

One TCP connection is kept per robot address, so several robots work too.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import struct

log = logging.getLogger("bridge")
Addr = tuple[str, int]


class Session:
    """TCP connection to the agent on behalf of one UDP client."""

    def __init__(self, udp: asyncio.DatagramTransport, client: Addr, host: str, port: int) -> None:
        self.udp = udp
        self.client = client
        self.host = host
        self.port = port
        self.writer: asyncio.StreamWriter | None = None
        self.ready = asyncio.Event()
        self.closed = False

    async def run(self) -> None:
        try:
            reader, self.writer = await asyncio.open_connection(self.host, self.port)
        except OSError as exc:
            log.error("cannot reach agent at %s:%d: %s", self.host, self.port, exc)
            self.closed = True
            self.ready.set()
            return
        log.info("session opened for %s:%d", *self.client)
        self.ready.set()
        try:
            while True:
                header = await reader.readexactly(2)
                (length,) = struct.unpack("<H", header)
                payload = await reader.readexactly(length)
                self.udp.sendto(payload, self.client)
        except (asyncio.IncompleteReadError, ConnectionError) as exc:
            log.warning("agent closed session for %s:%d (%s)", *self.client, type(exc).__name__)
        finally:
            self.closed = True
            if self.writer:
                self.writer.close()

    async def send(self, data: bytes) -> None:
        await self.ready.wait()
        if self.closed or not self.writer:
            return
        self.writer.write(struct.pack("<H", len(data)) + data)
        await self.writer.drain()


class UdpSide(asyncio.DatagramProtocol):
    def __init__(self, agent_host: str, agent_port: int) -> None:
        self.agent_host = agent_host
        self.agent_port = agent_port
        self.sessions: dict[Addr, Session] = {}
        self.transport: asyncio.DatagramTransport | None = None

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    def datagram_received(self, data: bytes, addr: Addr) -> None:
        session = self.sessions.get(addr)
        if session is None or session.closed:
            assert self.transport is not None
            session = Session(self.transport, addr, self.agent_host, self.agent_port)
            self.sessions = {**{a: s for a, s in self.sessions.items() if not s.closed}, addr: session}
            asyncio.ensure_future(session.run())
        asyncio.ensure_future(session.send(data))


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--udp-port", type=int, default=8090, help="port the robot sends to")
    ap.add_argument("--agent-host", default="127.0.0.1")
    ap.add_argument("--agent-port", type=int, default=8888, help="agent tcp4 port")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(
        lambda: UdpSide(args.agent_host, args.agent_port), local_addr=("0.0.0.0", args.udp_port))
    log.info("UDP :%d <-> TCP %s:%d", args.udp_port, args.agent_host, args.agent_port)
    await asyncio.Event().wait()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
