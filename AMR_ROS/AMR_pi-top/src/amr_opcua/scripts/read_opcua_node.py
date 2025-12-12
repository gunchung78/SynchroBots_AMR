#!/usr/bin/env python3

import asyncio
import threading
from asyncua import Client, ua

import rospy
from std_msgs.msg import String

DEFAULT_OPCUA_URL = "opc.tcp://172.30.1.61:0630/freeopcua/server/"


class OpcuaSubHandler:
    """
    OPC UA subscription handler.
    Called whenever the subscribed node value changes.
    """

    def __init__(self, publisher, label: str):
        self._pub = publisher
        self._label = label  # for log messages

    def datachange_notification(self, node, val, data):
        """
        Called by OPC UA server on value change.
        Publishes the value as a ROS String message.
        """
        rospy.loginfo("[amr_opcua][READ] %s : %s", self._label, str(val))

        msg = String()
        msg.data = str(val)
        self._pub.publish(msg)


async def _opcua_subscription_loop(opcua_url: str,
                                   topic_name: str,
                                   node_id: str,
                                   label: str,
                                   queue_size: int):
    """
    Single OPC UA client + subscription loop for one node.
    This runs inside its own asyncio event loop (per thread).
    """
    pub = rospy.Publisher(topic_name, String, queue_size=queue_size)

    rospy.loginfo(
        "[amr_opcua][READ] Starting subscription: label=%s, node_id=%s, topic=%s",
        label,
        node_id,
        topic_name,
    )
    rospy.loginfo("[amr_opcua][READ] Connecting to OPC UA server: %s", opcua_url)

    async with Client(url=opcua_url) as client:
        # handler for this node
        handler = OpcuaSubHandler(pub, label)

        # create subscription
        sub = await client.create_subscription(100, handler)

        # get node by NodeId string
        node = client.get_node(node_id)

        # optional debug: read current value once
        try:
            current_val = await node.read_value()
            rospy.loginfo(
                "[amr_opcua][READ] %s initial value: %s",
                label,
                str(current_val),
            )
        except Exception as e:
            rospy.logwarn(
                "[amr_opcua][READ] Failed to read initial value for %s: %s",
                label,
                e,
            )

        # subscribe to data change
        await sub.subscribe_data_change(node)
        rospy.loginfo(
            "[amr_opcua][READ] Subscribed to node: %s (label=%s)",
            node_id,
            label,
        )

        try:
            while not rospy.is_shutdown():
                await asyncio.sleep(0.1)
        finally:
            rospy.loginfo(
                "[amr_opcua][READ] Shutting down subscription: label=%s",
                label,
            )
            try:
                await sub.delete()
            except Exception as e:
                rospy.logwarn(
                    "[amr_opcua][READ] Failed to delete subscription for %s: %s",
                    label,
                    e,
                )


def _opcua_thread_worker(opcua_url: str,
                         topic_name: str,
                         node_id: str,
                         label: str,
                         queue_size: int):
    """
    Thread target function.
    Each thread runs its own asyncio event loop to handle one OPC UA node.
    """
    try:
        asyncio.run(
            _opcua_subscription_loop(
                opcua_url=opcua_url,
                topic_name=topic_name,
                node_id=node_id,
                label=label,
                queue_size=queue_size,
            )
        )
    except Exception as e:
        rospy.logerr(
            "[amr_opcua][READ] Exception in OPC UA thread (label=%s): %s",
            label,
            e,
        )


def main():
    rospy.init_node("read_opcua_node", anonymous=False)

    opcua_url = rospy.get_param("~opcua_url", DEFAULT_OPCUA_URL)
    topic_move = rospy.get_param("~topic_move", "/amr/opcua/go_move")
    topic_positions = rospy.get_param("~topic_positions", "/amr/opcua/go_positions")
    queue_size = rospy.get_param("~queue_size", 10)

    rospy.loginfo("[amr_opcua][READ] Node started")
    rospy.loginfo("[amr_opcua][READ] OPC UA URL       : %s", opcua_url)
    rospy.loginfo("[amr_opcua][READ] Publish topic 1 : %s (read_amr_go_move)", topic_move)
    rospy.loginfo("[amr_opcua][READ] Publish topic 2 : %s (read_amr_go_positions)", topic_positions)

    # NodeIds for each variable
    node_id_move = "ns=2;s=read_amr_go_move"
    node_id_positions = "ns=2;s=read_amr_go_positions"

    # Create and start threads
    t_move = threading.Thread(
        target=_opcua_thread_worker,
        args=(opcua_url, topic_move, node_id_move, "read_amr_go_move", queue_size),
        daemon=True,
    )
    t_positions = threading.Thread(
        target=_opcua_thread_worker,
        args=(opcua_url, topic_positions, node_id_positions, "read_amr_go_positions", queue_size),
        daemon=True,
    )

    t_move.start()
    t_positions.start()

    # Keep ROS node alive
    rospy.spin()


if __name__ == "__main__":
    main()
