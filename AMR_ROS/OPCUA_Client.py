import asyncio
import json
from asyncua import Client

# ---------------------------------------------------------
# 1. OPC UA 서버 엔드포인트
#    👉 서버 코드에서 set_endpoint(...)에 적은 값과 "완전히 동일"해야 함
# ---------------------------------------------------------
OPC_ENDPOINT = "opc.tcp://172.30.1.61:0630/freeopcua/server/"

# ---------------------------------------------------------
# 2. 노드/메소드 정보
#    SynchroBots_OPCUA_Server.py 기준
# ---------------------------------------------------------
# ServerMethods.init_nodes() 에서 만든 Object 이름: "InterfaceDataNodes"
OBJECT_NODE_ID = "ns=2;i=1"

# AMR_003 Method 이름: 'amr_mission_state'
METHOD_BROWSENAME = "2:amr_mission_state"

# 서버가 상태를 써주는 Read 변수 노드
READBACK_NODE_ID = "ns=2;i=4"


async def send_mission_state(client, status: str):
    """
    AMR_003 (amr_mission_state) 메소드를 한 번 호출하고
    - 서버 응답 (result_code, result_message)
    - 상태 노드(read_amr_mission_state_status) 값
    을 출력하는 함수
    """
    # 임무 상태 JSON 예시 (원하면 mission_id만 바꿔도 됨)
    mission_state = {
        "equipment_id": "AMR_03",
        "mission_id": "2",
        "status": status
    }

    json_str = json.dumps(mission_state)

    # Object 노드 핸들
    obj = client.get_node(OBJECT_NODE_ID)

    print(f"\n[CALL] amr_mission_state(status='{status}')")
    result_code, result_message = await obj.call_method(
        METHOD_BROWSENAME,
        json_str
    )

    print("  - ResultCode   :", result_code)
    print("  - ResultMessage:", result_message)

    # 서버가 기록한 상태 노드를 읽어 확인
    readback_node = client.get_node(READBACK_NODE_ID)
    last_status = await readback_node.read_value()
    print("  - ReadbackNode :", last_status)


async def main():
    # OPC UA 서버 연결
    async with Client(OPC_ENDPOINT) as client:
        print(f"[INFO] Connected to OPC UA Server: {OPC_ENDPOINT}")

        # 테스트로 보낼 상태 시퀀스 (원하면 자유롭게 수정 가능)
        status_sequence = ["READY", "RUNNING", "DONE"]

        for s in status_sequence:
            await send_mission_state(client, s)
            await asyncio.sleep(2.0)  # 상태 사이 간격 2초 (`원하면 줄이거나 늘려도 됨)

        print("\n[INFO] AMR_003 1:1 테스트 종료")


if __name__ == "__main__":
    asyncio.run(main())