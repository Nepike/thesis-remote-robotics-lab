import asyncio
from typing import Dict, Type, Callable, Awaitable
from threading import Lock
from pathlib import Path

import rospy
import serial_asyncio


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

    def publish(self, topic: str, msg_type: Type, message):
        with self._lock:
            if topic not in self._publishers:
                self._publishers[topic] = rospy.Publisher(topic, msg_type, queue_size=10)
                rospy.loginfo(f"[RosInterface] Publisher created: {topic}")
            self._publishers[topic].publish(message)

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
