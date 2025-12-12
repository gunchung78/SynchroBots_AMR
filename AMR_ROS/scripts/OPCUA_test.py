import asyncio
from asyncua import Client, ua

OPCUA_URL = "opc.tcp://172.30.1.61:0630/freeopcua/server/"

class SubHandler:
    def datachange_notification(self, node, val, data):
        print("value :", val)
async def amr_subscriber():
    async with Client(OPCUA_URL) as client:
        handler = SubHandler()
        sub = await client.create_subscription(100, handler)
        cmd_node = await client.nodes.root.get_child([
            "0:Objects",
            "2:AMR",
            "2:read_amr_go_move"
        ])
        await sub.subscribe_data_change(cmd_node)
        while True:
            await asyncio.sleep(1)
if __name__ == "__main__":
    asyncio.run(amr_subscriber())