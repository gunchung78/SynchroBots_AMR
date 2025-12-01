import asyncio
import json
from asyncua import Client
# ---------------------------------------------------------
# 1. OPC UA 서버 엔드포인트
#    SynchroBots OPCUA Server.py 기준
# ---------------------------------------------------------
OPC_ENDPOINT = "opc.tcp://172.30.1.61:4840/freeopcua/server/"
# ---------------------------------------------------------
# 2. Method / Node 정보
# ---------------------------------------------------------
# 서버에서 등록한 Object는 "InterfaceDataNodes"
OBJECT_NODE_ID = "ns=2;s=InterfaceDataNodes"
# 메소드 이름은 그대로 "amr_mission_state"
METHOD_BROWSENAME = "2:amr_mission_state"
# 서버가 마지막 결과를 기록하는 Read Node
READBACK_NODE_ID = "ns=2;s=read_amr_mission_state_status"
# ---------------------------------------------------------
# 3. 메인 실행 함수
# ---------------------------------------------------------
async def main():
    # AMR이 서버에 전달하는 JSON 데이터 (예시)
    mission_state = {
        "equipment_id": "AMR_1",
        "mission_id": "AMR_1_251125",
        "status": "DONE"        # 서버에서 필수로 요구하는 key
    }
    json_str = json.dumps(mission_state)
    # ---------------------------------------------------------
    # OPC UA 서버 연결
    # ---------------------------------------------------------
    async with Client(OPC_ENDPOINT) as client:
        print(f"[INFO] Connected to OPC UA Server: {OPC_ENDPOINT}")
        # InterfaceDataNodes Object 가져오기
        obj = client.get_node(OBJECT_NODE_ID)
        print("[INFO] Calling method: amr_mission_state(...)")
        result_code, result_message = await obj.call_method(
            METHOD_BROWSENAME,
            json_str
        )
        print("\n===== SERVER RESPONSE =====")
        print(f"Result Code    : {result_code}")
        print(f"Result Message : {result_message}")
        # ---------------------------------------------------------
        # 서버에서 기록해 둔 결과 읽기
        # ---------------------------------------------------------
        readback_node = client.get_node(READBACK_NODE_ID)
        last_status = await readback_node.read_value()
        print("\n===== READBACK NODE VALUE =====")
        print(f"{READBACK_NODE_ID} → {last_status}")
if __name__ == "__main__":
    asyncio.run(main())