from comunication import Communication
from control import PID
from tracker import NutsTracker
import threading


if __name__ == '__main__':
    print("Starting...")
    tracker = NutsTracker()
    tracker.initiateVideo()
    t1 = threading.Thread(target = tracker.track)
    t1.start()

    print("Video Started!")
    coms = Communication()
    coms.begin()
    print("Coms Started!")
    control = PID(6, 0.03, 0.1, 295, 5, 5,round(tracker.x_max/2), round(tracker.y_max))
    print("Control Started!")
    try:
        while True:
            if(tracker.detect == 1):
                x = tracker.x
                y = tracker.y
                print(x,y)
                PWMA, PWMB = control.update(0.1, x, y)
                print(f"PWMA: {PWMA}, PWMB: {PWMB}")
                msg = f"{PWMA},{PWMB}\n"
                coms.comunicacion(msg)
            else:
                print("RPMA: 0, RPMB: 0")
                msg = "0,0\n" 
                coms.comunicacion(msg)
            coms.read_and_print_messages()
    
    except KeyboardInterrupt:
        print("Data collection interrupted.")

    finally:
        tracker.stopped = True
        t1.join()
    
