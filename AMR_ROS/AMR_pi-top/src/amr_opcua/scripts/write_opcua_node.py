#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import json

import rospy
from std_msgs.msg import String

from asyncua import Client

DEFAULT_OPCUA_URL = "opc.tcp://172.30.1.61:0630/freeopcua/server/"

OBJECT_NODE_ID = "ns=2;i=1"
METHOD_NODE_ID = "ns=2;i=17"


async def send_mission_state_once(status: str):
    opcua_url = rospy.get_param("~opcua_url", DEFAULT_OPCUA_URL)

    mission_state = {
        "status": status
    }
    json_str = json.dumps(mission_state)

    async with Client(opcua_url) as client:
        rospy.loginfo(f"[amr_opcua][WRITE] Connected to OPC UA server: {opcua_url}")

        obj = client.get_node(OBJECT_NODE_ID)
        method_node = client.get_node(METHOD_NODE_ID)

        rospy.loginfo(f"[amr_opcua][WRITE] CALL write_amr_mission_state(status='{status}')")

        result_code, result_message = await obj.call_method(
            method_node.nodeid,
            json_str
        )

        rospy.loginfo(f"[amr_opcua][WRITE] Result code   : {result_code}")
        rospy.loginfo(f"[amr_opcua][WRITE] Result message: {result_message}")


def amr_mission_state_callback(msg: String):
    status = msg.data.strip()
    if not status:
        rospy.logwarn("[amr_opcua][WRITE] Empty amr_mission_state received, ignore.")
        return

    rospy.loginfo(f"[amr_opcua][WRITE] Received amr_mission_state = '{status}'")

    try:
        asyncio.run(send_mission_state_once(status))
    except Exception as e:
        rospy.logerr(f"[amr_opcua][WRITE] Failed to send status via OPC UA: {e}")


def main():
    rospy.init_node("write_opcua_node", anonymous=False)
    rospy.loginfo("[amr_opcua][WRITE] write_opcua_node started")

    rospy.Subscriber("amr_mission_state", String,
                     amr_mission_state_callback, queue_size=10)

    rospy.loginfo("[amr_opcua][WRITE] Subscribing 'amr_mission_state' topic")
    rospy.spin()


if __name__ == "__main__":
    main()
