import asyncio
from asyncua import Client, ua
# 구독 핸들러 클래스 정의 (값이 변경될 때 호출됨)
class SubHandler:
    def datachange_notification(self, node, val, data):
        """
        Client B가 서버로부터 변경된 데이터를 실시간으로 받는 메서드입니다.
        """
        print(f"\n[SUBSCRIBED] Client B :별: 데이터 수신! 수신 값: {val}")
async def main():
    server_url = "opc.tcp://172.30.1.61:4840/freeopcua/server/"
    client = Client(url=server_url)
    # client.set_security(ua.SecurityPolicyType.NoSecurity)
    try:
        await client.connect()
        print("Client B 서버 접속 성공")
        uri = "http://examples.freeopcua.github.io"
        idx = await client.get_namespace_index(uri)
        # 서버로부터 데이터를 '받아올' 노드 탐색
        read_node = await client.nodes.root.get_child([
            "0:Objects",
            f"{idx}:MyObject",
            f"{idx}:ClientB_Read_Data"  # 구독 전용 노드
        ])
        print(f"구독 노드 ID 획득 성공: {read_node}")
        # 4. 구독 설정
        handler = SubHandler()
        sub = await client.create_subscription(500, handler) # 500ms Publishing Interval
        # 모니터링할 아이템 등록
        await sub.subscribe_data_change(read_node)
        print(":흰색_확인_표시: Client B 구독이 성공적으로 등록되었습니다. Client A의 데이터를 기다립니다.")
        # 5. 메인 루프 (연결 유지 및 수신 대기)
        while True:
            # 구독 연결을 유지하기 위해 무한 대기
            await asyncio.sleep(1)
    except Exception as e:
        print(f"\nClient B OPC UA 통신 오류 발생: {e}")
    finally:
        if 'sub' in locals() and sub:
            await sub.delete()
        if client:
            await client.disconnect()
            print("Client B 접속 종료")
if __name__ == "__main__":
    asyncio.run(main())