import asyncio
import json
from asyncua import Client


OPC_ENDPOINT = "opc.tcp://172.30.1.61:0630/freeopcua/server/"

OBJECT_NODE_ID = "ns=2;i=1"
METHOD_NODE_ID = "ns=2;i=17"


async def send_mission_state(client, status: str):
    mission_state = {
        "status": status
    }
    json_str = json.dumps(mission_state)

    obj = client.get_node(OBJECT_NODE_ID)      # Object: InterfaceDataNodes
    method_node = client.get_node(METHOD_NODE_ID)  # Method: write_amr_mission_state

    print(f"\n[CALL] amr_mission_state(status='{status}')")

    result_code, result_message = await obj.call_method(
        method_node.nodeid,
        json_str
    )

    print("  - ResultCode   :", result_code)
    print("  - ResultMessage:", result_message)


async def main():
    async with Client(OPC_ENDPOINT) as client:
        print(f"[INFO] Connected to OPC UA Server: {OPC_ENDPOINT}")

        status_sequence = ["PICK", "ERROR"]

        for s in status_sequence:
            await send_mission_state(client, s)
            await asyncio.sleep(2.0)

        print("\n[INFO] AMR_003 1:1 테스트 종료")


if __name__ == "__main__":
    asyncio.run(main())
