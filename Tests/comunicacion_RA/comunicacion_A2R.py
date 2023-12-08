import serial

ser = serial.Serial('/dev/ttyACM0',9600)
ser.flushInput()

while True:
    # print('hola')
    try:
        lineBytes = ser.readline()
        line = lineBytes.decode('utf-8').strip()
        print(line)
    except KeyboardInterrupt:
        break