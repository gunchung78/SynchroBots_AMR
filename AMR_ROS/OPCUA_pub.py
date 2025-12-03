import asyncio
from asyncua import Client, ua
async def main():
    server_url = "opc.tcp://172.30.1.61:4840/freeopcua/server/"
    client = Client(url=server_url)
    # client.set_security(ua.SecurityPolicyType.NoSecurity)
    try:
        await client.connect()
        print("Client A 서버 접속 성공")
        uri = "http://examples.freeopcua.github.io"
        idx = await client.get_namespace_index(uri)
        # 서버에 값을 보낼 노드 탐색
        write_node = await client.nodes.root.get_child([
            "0:Objects",
            f"{idx}:MyObject",
            f"{idx}:ClientA_Write_Data"  # 쓰기 전용 노드
        ])
        print(f"쓰기 노드 ID 획득 성공: {write_node}")
        # 4. 데이터 쓰기 루프
        value_to_send = 0
        while True:
            value_to_send += 10
            await write_node.write_value(value_to_send)
            print(f"Client A :로켓: 서버로 데이터 전송 (ClientA_Write_Data): {value_to_send}")
            await asyncio.sleep(1) # 1초마다 데이터 전송
    except Exception as e:
        print(f"Client A OPC UA 통신 오류 발생: {e}")
    finally:
        if client:
            await client.disconnect()
            print("Client A 접속 종료")
if __name__ == "__main__":
    asyncio.run(main())