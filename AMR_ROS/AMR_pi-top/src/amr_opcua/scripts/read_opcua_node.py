#!/usr/bin/env python3

import asyncio
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
        msg.data = str(val)  # JSON string or plain string
        self._pub.publish(msg)


async def amr_opcua_main():
    # Initialize ROS node
    rospy.init_node("read_opcua_node", anonymous=False)

    # Parameters (can be overridden in launch file)
    opcua_url = rospy.get_param("~opcua_url", DEFAULT_OPCUA_URL)
    topic_move = rospy.get_param("~topic_move", "/amr/opcua/go_move")
    topic_positions = rospy.get_param("~topic_positions", "/amr/opcua/go_positions")
    queue_size = rospy.get_param("~queue_size", 10)

    pub_move = rospy.Publisher(topic_move, String, queue_size=queue_size)
    pub_positions = rospy.Publisher(topic_positions, String, queue_size=queue_size)

    rospy.loginfo("[amr_opcua][READ] OPC UA URL       : %s", opcua_url)
    rospy.loginfo("[amr_opcua][READ] Publish topic 1 : %s (read_amr_go_move)", topic_move)
    rospy.loginfo("[amr_opcua][READ] Publish topic 2 : %s (read_amr_go_positions)", topic_positions)
    rospy.loginfo("[amr_opcua][READ] Connecting to OPC UA server...")

    # Connect to OPC UA server
    async with Client(url=opcua_url) as client:
        # Example security setting (disabled here):
        # await client.set_security(ua.SecurityPolicyType.NoSecurity)

        # Two handlers (each publishes to a different topic)
        handler_move = OpcuaSubHandler(pub_move, "read_amr_go_move")
        handler_positions = OpcuaSubHandler(pub_positions, "read_amr_go_positions")

        # Two subscriptions (100 ms interval)
        sub_move = await client.create_subscription(100, handler_move)
        sub_positions = await client.create_subscription(100, handler_positions)

        # 1) AMR/read_amr_go_move node
        cmd_node = await client.nodes.root.get_child([
            "0:Objects",
            "2:AMR",
            "2:read_amr_go_move"
        ])
        await sub_move.subscribe_data_change(cmd_node)
        rospy.loginfo("[amr_opcua][READ] Subscribed to node: AMR/read_amr_go_move")

        # 2) AMR/read_amr_go_positions node
        pos_node = await client.nodes.root.get_child([
            "0:Objects",
            "2:AMR",
            "2:read_amr_go_positions"
        ])
        await sub_positions.subscribe_data_change(pos_node)
        rospy.loginfo("[amr_opcua][READ] Subscribed to node: AMR/read_amr_go_positions")

        # Keep the loop alive while ROS is running
        try:
            while not rospy.is_shutdown():
                await asyncio.sleep(0.1)
        finally:
            rospy.loginfo("[amr_opcua][READ] Shutting down OPC UA subscription...")
            try:
                await sub_move.delete()
            except Exception as e:
                rospy.logwarn("[amr_opcua][READ] Failed to delete move subscription: %s", e)
            try:
                await sub_positions.delete()
            except Exception as e:
                rospy.logwarn("[amr_opcua][READ] Failed to delete positions subscription: %s", e)


def main():
    try:
        asyncio.run(amr_opcua_main())
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass


if __name__ == "__main__":
    main()
