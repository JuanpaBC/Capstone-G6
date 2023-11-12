response = self.arduino.readline().decode('utf-8').strip()
            print(response)
            if response == "ACK":
                print("Arduino received the message")
            else:
                print("Arduino did not acknowledge the message")
