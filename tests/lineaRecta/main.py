from comunication import Communication
from control import PID
from tracker import NutsTracker


if __name__ == '__main__':
    print("Starting...")
    print("Video Started!")
    coms = Communication()
    coms.begin()
    print("Coms Started!")

    RPMA = 0
    RPMB = 0
    posX = 0
    try:
        while True:
            if(int(round(float(posX))) < 1):
                msg = "90,90\n"
            else:
                msg = "0,0\n" 
            coms.comunicacion(msg)
            coms.read_and_print_messages()
            if(coms.data != ""):
                splitData = coms.data.split(',')
                timestamp = splitData[0]
                posX = splitData[1].split(':')[1]
    
    except KeyboardInterrupt:
        print("Data collection interrupted.")
        coms.close()
