import asyncio
import json
import serial
import time
import websockets

clients = set()

async def broadcast(message):
    if clients:
        await asyncio.gather(*[client.send(message) for client in clients])
        
async def handler(websocket):
    clients.add(websocket)
    print("Client connected")
    
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)
        print("Client disconnected")
        
async def serial_reader():
    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=115200,
        timeout=0.1,
    )
    
    while True:
        line = ser.readline().decode('utf-8').strip()

        if line:
            parts = line.split(',')
            
            if len(parts) == 7:
                data = {
                    'ax': float(parts[0]),
                    'ay': float(parts[1]),
                    'az': float(parts[2]),
                    'temp': float(parts[3]),
                    'gx': float(parts[4]),
                    'gy': float(parts[5]),
                    'gz': float(parts[6]),
                }
                
                await broadcast(json.dumps(data))
            
            await asyncio.sleep(0.01)

async def main():
    server = await websockets.serve(
        handler,
        '0.0.0.0',
        8765,
    )
    
    await serial_reader()

if __name__ == '__main__':
    asyncio.run(main())