import asyncio
import json

async def main():
    reader, writer = await asyncio.open_connection("127.0.0.1", 9000)
    writer.write((json.dumps({"T": 1, "L": 0.2, "R": 0.2}) + "\n").encode())
    await writer.drain()
    while True:
        line = await reader.readline()
        if not line:
            break
        print(line.decode().strip())

asyncio.run(main())