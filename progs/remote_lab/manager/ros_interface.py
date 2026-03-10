import rospy
from typing import Dict, Type, Callable
from threading import Lock


class RosInterface:
    def __init__(self, node_name: str = "remotelab_manager"):
        rospy.init_node(node_name, anonymous=True)

        self._publishers: Dict[str, rospy.Publisher] = {}
        self._subscribers: Dict[str, rospy.Subscriber] = {}

        self._lock = Lock()

        rospy.loginfo("[RosInterface] initialized")

    def _get_publisher(self, topic: str, msg_type: Type):
        with self._lock:
            if topic not in self._publishers:
                pub = rospy.Publisher(
                    topic,
                    msg_type,
                    queue_size=10
                )
                self._publishers[topic] = pub
                rospy.loginfo(f"[RosInterface] Publisher created: {topic}")
            return self._publishers[topic]

    def publish(self, topic: str, msg_type: Type, message):
        pub = self._get_publisher(topic, msg_type)
        pub.publish(message)

    def subscribe(self, topic: str, msg_type: Type, callback: Callable):
        with self._lock:
            if topic in self._subscribers:
                rospy.logwarn(f"[RosInterface] already subscribed: {topic}")
                return

            sub = rospy.Subscriber(
                topic,
                msg_type,
                callback,
                queue_size=50
            )
            self._subscribers[topic] = sub
            rospy.loginfo(f"[RosInterface] Subscribed: {topic}")

    def unsubscribe(self, topic: str):
        with self._lock:
            if topic not in self._subscribers:
                return

            sub = self._subscribers.pop(topic)
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