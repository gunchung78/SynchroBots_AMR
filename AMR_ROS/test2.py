import asyncio
from asyncua import Client
import json
# ===== 0. 환경 설정 =====
# :오른쪽을_가리키는_손_모양: 실제 OPC UA 서버 endpoint 로 바꿔줘
SERVER_URL = "opc.tcp://172.30.1.61:0630/freeopcua/server/"
# :오른쪽을_가리키는_손_모양: 엑셀 정의서 기준 (예시)
# AMR_001 오브젝트 Node ID (Object Node)
OBJECT_NODE_ID = "ns=2;i=2"      # 실제 AMR_001 오브젝트 ID로 변경 가능
# read_amr_go_move 메소드 Node ID
READ_METHOD_NODE_ID = "ns=2;i=4" # 엑셀에 있던 Method ID(OUTPUT)
async def handle_move_command(json_str: str):
    """
    read_amr_go_move가 반환한 JSON 문자열을 해석해서
    실제 AMR 이동 로직을 호출하는 부분.
    """
    print(f"[AMR][CLIENT] raw json command = {json_str!r}")
    if not json_str:
        print("[AMR][CLIENT] 빈 명령이라 무시")
        return
    try:
        data = json.loads(json_str)
    except json.JSONDecodeError as e:
        print(f"[AMR][CLIENT][ERROR] JSON 파싱 실패: {e}")
        return
    move_cmd = data.get("move_command")
    print(f"[AMR][CLIENT] >>> 새 move_command 수신: {move_cmd!r}")
    # TODO: 여기서 실제 AMR 제어 코드 호출
    # 예:
    #   if move_cmd == "go_home":
    #       await amr_go_home()
    #   elif move_cmd == "pick_up_zone":
    #       await amr_go_pick_up_zone()
    #
    # 지금은 데모라 print만 사용
async def main():
    print("[AMR][CLIENT] AMR_001 OPC UA Client 시작")
    async with Client(url=SERVER_URL) as client:
        print(f"[AMR][CLIENT] 서버 접속 완료: {SERVER_URL}")
        # AMR_001 오브젝트 노드 / read_amr_go_move 메소드 노드
        obj_node = client.get_node(OBJECT_NODE_ID)
        method_node = client.get_node(READ_METHOD_NODE_ID)
        last_cmd = None
        while True:
            try:
                # read_amr_go_move 호출 (입력 인자 없음 가정)
                # 서버 메소드가 [Boolean, String] 형식으로 돌려준다고 가정
                result = await obj_node.call_method(method_node)
                has_cmd, json_str = result  # (True/False, JSON 문자열)
                if not has_cmd:
                    # 현재 내려온 명령 없음
                    await asyncio.sleep(0.5)
                    continue
                # 같은 명령이면 스킵(필요 없으면 이 if 블록 지워도 됨)
                if json_str == last_cmd:
                    await asyncio.sleep(0.5)
                    continue
                last_cmd = json_str
                await handle_move_command(json_str)
            except Exception as e:
                print(f"[AMR][CLIENT][ERROR] read_amr_go_move 호출 중 예외: {e}")
                # 에러 시 잠깐 쉬었다가 재시도
                await asyncio.sleep(1.0)
if __name__ == "__main__":
    asyncio.run(main())