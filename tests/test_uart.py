import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)

print(ser.read(50))

ser.close()