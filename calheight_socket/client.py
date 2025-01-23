import asyncio
import json
import websockets

async def client():
    uri = "ws://localhost:8080"  # 替换为你的服务器地址和端口
    async with websockets.connect(uri) as websocket:
        # 模拟要发送的JSON消息
        message = json.dumps([
            {"X": 4.915782, "Y": 1.1711626, "Z": 41.76108},
            {"X": 4.925597, "Y": 1.187583, "Z": 42.03227},
            {"X": 6.833519, "Y": 1.3667307, "Z": 52.26195},
            {"X": 4.908695, "Y": 1.2518072, "Z": 51.74423},
            {"X": 4.8877044, "Y": 1.185895, "Z": 44.0346},
            {"X": 8.4, "Y": 4.48, "Z": 45.4509 },
            {"X": 4.913353, "Y": 1.3619995, "Z": 46.72006}
        ])
        await websocket.send(message)
        # print(f"> Sent: {message}")

        # 接收服务器返回的结果
        response = await websocket.recv()
        print(f"< Received: {response}")

# 运行客户端
asyncio.get_event_loop().run_until_complete(client())
