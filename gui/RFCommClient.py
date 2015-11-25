import bluetooth


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

    def connect_disconnect_button(self):
        if self.status == "NOT CONNECTED":
            while not self.connect():
                pass
        else:
            self.disconnect()

    def set_host_address(self, baddr):
        print("hej")
        self.host = baddr


class RfCommClient:
    def __init__(self, baddr, port):
        self.host = baddr   #macadressen till målet/hosten
        self.port = port    #porten, 1 funkar typ
        self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
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

    def connect_disconnect_button(self):
        if self.status == "NOT CONNECTED":
            while not self.connect():
                pass
        else:
            self.disconnect()
