import bluetooth


class RfCommClient():
    def __init__(self, baddr, port):
        self.host = baddr   #macadressen till målet/hosten
        self.port = port    #porten, 1 funkar typ
        self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    def __del__(self):
        self.socket.close()
        class_name = self.__class__.__name__
        print(class_name, "destroyed")

    def connect(self):
        try:
            self.socket.connect((self.host, self.port))
            print("Connected to: " + self.host)
            return True
        except IOError:
            print("Connection timed out")
            return False

    def send(self, data):
        if self.host is not None:
            self.socket.send(data)
            print("Sent: " + str(data))

    def receive(self):
        if self.host is not None:
            data = self.socket.recv(1)
            print("Received:  " + str(data))
            return data
        return -1
