
except KeyboardInterrupt:
    print("Data collection interrupted.")

finally:
    # Close the serial port
    coms.arduino.close()