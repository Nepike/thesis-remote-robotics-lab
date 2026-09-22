import asyncio
import socket
from typing import Dict, Optional, Type, Callable, Awaitable
from threading import Lock
from pathlib import Path

import rospy
import serial_asyncio

from Logger import Logger


class RosInterface:
    """
    Manages ROS publishers and subscribers for all connected ROS-based devices.

    VNIMANIE!: rospy delivers messages in its own internal threads.
    Use subscribe() for plain sync callbacks, subscribe_async() for async callbacks
    that need to run in the asyncio event loop (e.g. for telemetry forwarding).
    """

    def __init__(self, node_name: str = "remotelab_manager"):
        rospy.init_node(node_name, anonymous=True)

        self._publishers: Dict[str, rospy.Publisher] = {}
        self._subscribers: Dict[str, rospy.Subscriber] = {}
        self._lock = Lock()

        rospy.loginfo("[RosInterface] initialized")

    def register_publisher(self, topic: str, msg_type: Type) -> rospy.Publisher:
        """
        Create the publisher for a topic if it does not exist yet, WITHOUT sending
        a message. Returns the publisher.

        Used to pre-create (warm up) publishers ahead of the first command: rospy
        establishes publisher-subscriber connections asynchronously, so a message
        published in the same instant the publisher is created is silently dropped.
        Creating the publisher early decouples creation from the first publish.
        """
        with self._lock:
            if topic not in self._publishers:
                self._publishers[topic] = rospy.Publisher(topic, msg_type, queue_size=10)
                rospy.loginfo(f"[RosInterface] Publisher created: {topic}")
            return self._publishers[topic]

    def publish(self, topic: str, msg_type: Type, message):
        pub = self.register_publisher(topic, msg_type)
        pub.publish(message)

    async def wait_for_publisher(self, topic: str, timeout: float = 3.0, poll: float = 0.05) -> bool:
        """
        Wait (best-effort) until the publisher for `topic` has at least one
        connected subscriber, or until `timeout` seconds elapse.

        Returns True if a connection was established. Non-fatal: on timeout the
        caller proceeds anyway (the publisher already exists, so a later command
        will not hit the create+publish race).
        """
        with self._lock:
            pub = self._publishers.get(topic)
        if pub is None:
            return False

        waited = 0.0
        while waited < timeout:
            if pub.get_num_connections() > 0:
                return True
            await asyncio.sleep(poll)
            waited += poll
        return pub.get_num_connections() > 0

    def subscribe(self, topic: str, msg_type: Type, callback: Callable):
        """Subscribe with a synchronous callback (runs in rospy's thread)."""
        with self._lock:
            if topic in self._subscribers:
                rospy.logwarn(f"[RosInterface] Already subscribed: {topic}")
                return
            self._subscribers[topic] = rospy.Subscriber(topic, msg_type, callback, queue_size=50)
            rospy.loginfo(f"[RosInterface] Subscribed: {topic}")

    def subscribe_async(self, topic: str, msg_type: Type, callback: Callable[..., Awaitable[None]]):
        """
        Subscribe with an async callback.

        Rospy delivers messages in its own threads; this method bridges them to the asyncio
        event loop via run_coroutine_threadsafe. Use this for telemetry: the callback is
        scheduled on the event loop each time a message arrives.
        """
        loop = asyncio.get_running_loop()
        def _bridge(msg):
            asyncio.run_coroutine_threadsafe(callback(msg), loop)

        self.subscribe(topic, msg_type, _bridge)

    def unsubscribe(self, topic: str):
        with self._lock:
            sub = self._subscribers.pop(topic, None)
            if sub:
                sub.unregister()
                rospy.loginfo(f"[RosInterface] Unsubscribed: {topic}")

    def shutdown(self):
        rospy.loginfo("[RosInterface] shutting down")
        with self._lock:
            for sub in self._subscribers.values():
                sub.unregister()
            for pub in self._publishers.values():
                pub.unregister()
            self._subscribers.clear()
            self._publishers.clear()


class SerialInterface:
    """
    Manages async serial connections to TTY ports.

    Each device has its own connection, identified by its TTY path.
    The interface is shared across all SerialBasedDrivers — each driver opens
    its own port via open() and interacts through write()/readline().

    For telemetry, subscribe_async() starts a background reader loop that calls
    the given callback for each received line.
    """

    class _Connection:
        def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
            self.reader = reader
            self.writer = writer

    def __init__(self):
        self._connections: Dict[str, SerialInterface._Connection] = {}
        self._telemetry_tasks: Dict[str, asyncio.Task] = {}

    async def open(self, port: Path, baud_rate: int):
        """Open an async serial connection to the given TTY port."""
        key = str(port)
        if key in self._connections:
            return

        reader, writer = await serial_asyncio.open_serial_connection(url=key, baudrate=baud_rate)
        self._connections[key] = SerialInterface._Connection(reader, writer)

    async def close(self, port: Path):
        """Close the serial connection and stop any running telemetry loop."""
        key = str(port)

        task = self._telemetry_tasks.pop(key, None)
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

        conn = self._connections.pop(key, None)
        if conn:
            conn.writer.close()
            await conn.writer.wait_closed()

    async def write(self, port: Path, data: bytes):
        """Write bytes to the device. The caller is responsible for framing (e.g. adding \\n)."""
        conn = self._connections.get(str(port))
        if conn is None:
            raise RuntimeError(f"Serial port not open: {port}")
        conn.writer.write(data)
        await conn.writer.drain()

    async def readline(self, port: Path) -> bytes:
        """Read one newline-terminated line from the device."""
        conn = self._connections.get(str(port))
        if conn is None:
            raise RuntimeError(f"Serial port not open: {port}")
        return await conn.reader.readline()

    def subscribe_async(self, port: Path, callback: Callable[[bytes], Awaitable[None]]):
        """
        Start a background reader loop that calls callback for each line received.

        Intended for serial telemetry. The loop runs until close() is called.
        Calling subscribe_async() twice for the same port is a no-op.
        """
        key = str(port)
        if key in self._telemetry_tasks:
            return
        self._telemetry_tasks[key] = asyncio.create_task(self._reader_loop(port, callback))

    async def _reader_loop(self, port: Path, callback: Callable[[bytes], Awaitable[None]]):
        try:
            while True:
                line = await self.readline(port)
                if not line:
                    break
                await callback(line)
        except asyncio.CancelledError:
            pass


class TcpInterface:
    """
    Manages persistent line-oriented TCP connections to network-native devices.

    For a device whose controller IS the network endpoint (an ESP32 that runs the
    robot itself, rather than bridging to an Arduino), there is nothing to gain
    from the socat path that RosInterface / SerialInterface use: turning a TCP
    stream into a pty only to have asyncio read it back as a serial port is a
    round trip through the kernel for no benefit, plus one more process to
    supervise. Here we just open the socket.

    Reconnection is the interface's own job. DeviceSupervisor watches PROCESSES,
    and a driver built on this one has none — so instead of dying and being
    restarted from outside, each link runs a supervised loop that reconnects on
    its own. A device that is switched off simply has a link that keeps retrying;
    writes to it raise ConnectionError until it comes back.

    Each link is identified by an arbitrary key (drivers pass the device name).
    """

    # How long to wait for a TCP handshake before giving up and retrying.
    CONNECT_TIMEOUT: float = 5.0
    # Pause between reconnect attempts. Short enough that a rebooting robot is
    # back in the fleet quickly, long enough not to spin on a powered-off one.
    RECONNECT_DELAY: float = 2.0

    class _Link:
        def __init__(self, host: str, port: int, callback: Callable[[bytes], Awaitable[None]]):
            self.host = host
            self.port = port
            self.callback = callback
            self.writer: Optional[asyncio.StreamWriter] = None
            self.task: Optional[asyncio.Task] = None

    def __init__(self):
        self._links: Dict[str, TcpInterface._Link] = {}

    async def open(self, key: str, host: str, port: int, callback: Callable[[bytes], Awaitable[None]]):
        """
        Start (and keep) a connection to host:port, calling `callback` for every
        received line. Returns immediately — the first connection happens in the
        background, so a device that is currently off does not block startup.
        """
        if key in self._links:
            return
        link = TcpInterface._Link(host, port, callback)
        self._links[key] = link
        link.task = asyncio.create_task(self._link_loop(key, link), name=f"tcp-link-{key}")

    async def close(self, key: str):
        """Stop the link loop and drop the connection. Safe to call twice."""
        link = self._links.pop(key, None)
        if link is None:
            return

        if link.task:
            link.task.cancel()
            try:
                await link.task
            except asyncio.CancelledError:
                pass
        await self._close_writer(link.writer)
        link.writer = None

    def is_connected(self, key: str) -> bool:
        link = self._links.get(key)
        return bool(link and link.writer is not None and not link.writer.is_closing())

    async def write(self, key: str, data: bytes):
        """
        Send bytes to the device. The caller owns framing (i.e. adds the newline).

        Raises ConnectionError while the device is unreachable. That is deliberate:
        the command worker logs the failure and the client is told the command
        settled, instead of a move silently "succeeding" against a robot that is
        switched off.
        """
        link = self._links.get(key)
        if link is None:
            raise RuntimeError(f"TCP link not open: {key}")

        writer = link.writer
        if writer is None or writer.is_closing():
            raise ConnectionError(f"Device '{key}' ({link.host}:{link.port}) is not connected")

        try:
            writer.write(data)
            await writer.drain()
        except OSError as e:
            # The link loop will notice and reconnect; report it as a lost
            # connection rather than leaking a raw socket error upwards.
            raise ConnectionError(f"Write to '{key}' failed: {e}") from e

    @staticmethod
    async def _close_writer(writer: Optional[asyncio.StreamWriter]):
        if writer is None:
            return
        try:
            writer.close()
            await writer.wait_closed()
        except OSError:
            pass  # already gone — nothing to clean up

    async def _link_loop(self, key: str, link: "TcpInterface._Link"):
        """Connect, pump lines until the peer goes away, wait, repeat."""
        logger = Logger.get()
        announced_failure = False

        while True:
            try:
                reader, writer = await asyncio.wait_for(
                    asyncio.open_connection(link.host, link.port),
                    timeout=self.CONNECT_TIMEOUT,
                )
            except (OSError, asyncio.TimeoutError) as e:
                # Log the first failure only: a robot that is off would otherwise
                # print a line every RECONNECT_DELAY seconds, forever.
                if not announced_failure:
                    announced_failure = True
                    await logger.log("TCP", f"'{key}' unreachable ({link.host}:{link.port}): {e}")
                await asyncio.sleep(self.RECONNECT_DELAY)
                continue

            announced_failure = False
            sock = writer.get_extra_info("socket")
            if sock is not None:
                try:
                    # Same reasoning as setNoDelay() on the ESP32 side: for
                    # real-time control, latency beats channel efficiency.
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass

            link.writer = writer
            await logger.log("TCP", f"'{key}' connected to {link.host}:{link.port}")

            try:
                while True:
                    line = await reader.readline()
                    if not line:
                        break  # EOF — the device closed the connection
                    try:
                        await link.callback(line)
                    except Exception as e:
                        # A malformed telemetry line must not tear down the
                        # transport. Log and keep the link.
                        await logger.log("TCP", f"'{key}' telemetry callback failed: {e}")
            except OSError:
                pass
            finally:
                link.writer = None
                await self._close_writer(writer)

            await logger.log("TCP", f"'{key}' disconnected, reconnecting")
            await asyncio.sleep(self.RECONNECT_DELAY)
