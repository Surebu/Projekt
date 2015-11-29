import bluetooth
import random

class FakeRFCommClient:
    def __init__(self, baddr, port):
        self.host = baddr   #macadressen till målet/hosten
        self.port = port    #porten, 1 funkar typ
        self.socket = False
        self.status = "NOT CONNECTED"

    def send(self, data):
        if self.host is not None:
            print("Sent: " + str(data))

    def connect(self):
        self.socket = True
        self.status = "CONNECTED"
        print(self.socket)
        return True

    def disconnect(self):
        self.socket = False
        self.status = "NOT CONNECTED"
        print(self.status)
        return True

    def receive(self):
        if self.host is not None:
            data = random.randint(0, 255)
            print("Received:  " + str(data))
            return data
        return -1

    def set_host_address(self, baddr):
        print("hej")
        self.host = baddr

## RfCommClient är bara ett enklare interface med PyBluez med bara nödvändiga funktioner samt funktioner för att hjälpa
## GUI:t
class RfCommClient:
    def __init__(self, baddr, port):
        self.host = baddr   #macadressen till målet/hosten
        self.port = port    #porten, 1 funkar typ
        self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)   #öppna en socket som sedan kan ansluta till roboten
        self.status = "NOT CONNECTED"

    def __del__(self):
        self.disconnect()
        class_name = self.__class__.__name__
        print(class_name, "destroyed")

    def connect(self):
        try:
            self.socket.connect((self.host, self.port))
            self.status = "CONNECTED"
            print("Connected to: " + self.host)
            return True
        except IOError:
            self.status = "NOT CONNECTED"
            print("Connection timed out")
            return False

    def disconnect(self):
        self.socket.close()
        self.status = "NOT CONNECTED"

    # Skickar en byte till hosten(roboten)
    def send(self, data):
        if self.host is not None:
            self.socket.send(data)
            print("Sent: " + str(data))

    # Tar emot en byte från hosten(roboten)
    def receive(self):
        if self.host is not None:
            data = self.socket.recv(1)
            print("Received:  " + str(data))
            return data
        return -1
