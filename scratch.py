
import time
import serial
import threading

def recibir():
    while True:
        line = ser.readline().decode('utf-8').rstrip()
        if line:
            print("\nArduino says: {}".format(line))

def enviar():
    while True:
        msg = "1,200,200\n"
        if msg == "exit":
            break
        ser.write(msg.encode('utf-8'))
        time.sleep(0.1)

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
    ser.flush()
    time.sleep(1)

    print("Rasp says: Let's start!")


    if ser.isOpen():
        print("{} conectado!".format(ser.port))
        read_messages_thread = threading.Thread(target=recibir, args=())
        read_messages_thread.start()
        send_messages_thread = threading.Thread(target=enviar, args=())
        send_messages_thread.start()

        send_messages_thread.join()
        read_messages_thread.join()
    print("\nRasp says: OK, let's stop here!")