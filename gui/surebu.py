from misc import *
from RFCommClient import *

LIFE = pygame.image.load("heart60.png")
LIFE_RECT = LIFE.get_rect()


class Surebu:
    """
    Container som håller alla värden från roboten och uppkopplingen till den. Är också en kontroller i den mån att den
    skickar kommandon och förfrågningar till roboten över uppkopplingen.
    """
    def __init__(self):
        self.data = {
            "IR-sensor 1 V": 0,
            "IR-sensor 2 B": 0,
            "IR-sensor 3 F": 0,
            "IR-sensor 4 H": 0,
            "Tejpsensor 1 VF": 0,
            "Tejpsensor 2 VB": 0,
            "Tejpsensor 3 HB": 0,
            "Tejpsensor 4 HF": 0,
            "Avståndssensor": 0,
            "Träffdetektor": 0,
            "Tapevalues":0,
            "Liv": 3,
            "Kontrolläge":0,
            "Move":0
        }
        self.dataAddress = 0
        self.rfClient = RFCommClient(SUREBU1_MACADDR, 1)

    def change_control_mode(self):
        """
        Byter mellan att roboten ska styras av programmet på datorn eller vara autonom.
        """
        self.rfClient.send(CHANGE_CONTROL_MODE)
        if self.data["Kontrolläge"] is 1:
            self.data["Kontrolläge"] = 0
        else:
            self.data["Kontrolläge"] = 1

    def connect_disconnect_button(self):
        """
        Försöker antingen ansluta till eller koppla ifrån roboten. Om metoden ska ansluta till roboten så skickar den
        även en förfrågan om vilket kontrolläge roboten är i och  väntar sedan på svar.
        """
        if self.rfClient.status == "NOT CONNECTED":
            tries = 0
            while not self.rfClient.connect():
                tries += 1
                if tries is 10:
                    break
                else:
                    pass
            if self.rfClient.status == "CONNECTED":
                self.rfClient.send(bytes([ADDRESSES.index("Kontrolläge")]))
                self.data["Kontrolläge"] = self.rfClient.receive()
            else:
                print("Failed to connect to: " + self.rfClient.host)
        else:
            self.rfClient.disconnect()

    def draw(self, screen, dims):
        """
        Tar in en yta och dimensioner/förhållanden och ritar ut sensorvärden och
        status för uppkopplingen till roboten
        """
        i = 0
        for address in ADDRESSES:
            if i < 10:
                text = FONT.render(address + ": " + str(self.data[address]), 5, BLACK)
                if i < 4:
                    screen.blit(text, (dims.widthPadding / 4, dims.heightPadding + 30*i))
                elif i < 8:
                    screen.blit(text, (dims.buttonWidth, dims.heightPadding + 30*(i - 4)))
                elif i is 8:
                    screen.blit(text, (dims.widthPadding / 4, dims.totalHeight / 2.2))
                elif i is 9:
                    screen.blit(text, (dims.widthPadding / 4, dims.totalHeight / 2.2 + dims.heightPadding))
            elif i is 10:
                text = FONT.render(address + ": " + str('{0:04b}'.format(self.data[address]))[::-1],5,BLACK)
                screen.blit(text,(dims.widthPadding / 4, dims.totalHeight / 2.2 + 2*dims.heightPadding))
            elif i is 13:
                text = FONT.render(address + ": " + str(hex(self.data[address])), 5, BLACK)
                screen.blit(text, (dims.widthPadding / 4, dims.totalHeight / 2.2 + 3*dims.heightPadding))
            elif i is 11:
                text = FONT.render(address + ": ", 5, BLACK)
                screen.blit(text, (dims.widthPadding / 4, dims.totalHeight / 2.2 + 4*dims.heightPadding))
                for j in range(self.data[address]):
                    screen.blit(LIFE, (dims.widthPadding / 4 + j * (LIFE_RECT.width + dims.widthPadding / 4),
                                       dims.totalHeight / 2.2 + 5*dims.heightPadding))
            i += 1
        self.__draw_status(screen, dims)

    def __draw_status(self, screen, dims):
        """
        Hjälpmetod för att rita ut statusen för uppkopplingen till roboten och i vilket kontrolllägen roboten är i.
        """
        status = SMALL_FONT.render("STATUS: ", 5, BLACK)

        if self.rfClient.status is "CONNECTED":
            connection_satus = UNDERLINED_FONT.render(self.rfClient.status, 5, GREEN)
            #if self.data["Kontrolläge"] is 1:
            #    control_status = UNDERLINED_FONT.render("CONTROL", 5, GREEN)
            #else:
            #    control_status = UNDERLINED_FONT.render("AUTONOMOUS", 5, RED)
        else:
            connection_satus = UNDERLINED_FONT.render(self.rfClient.status, 5, RED)
            #control_status = SMALL_FONT.render("N/a", 5, RED)

        screen.blit(status, (dims.rightPanelStart + dims.widthPadding,
                            dims.buttonHeight + dims.heightPadding*2.5))
        screen.blit(connection_satus, (dims.rightPanelStart + 2.2*dims.widthPadding,
                            dims.buttonHeight + dims.heightPadding*2.5))
        #screen.blit(status, (dims.rightPanelStart + dims.widthPadding,
        #                    2*dims.buttonHeight + dims.heightPadding*4.5))
        #screen.blit(control_status, (dims.rightPanelStart + 2.2*dims.widthPadding,
        #                    2*dims.buttonHeight + dims.heightPadding*4.5))

    def control(self, events):
        """
        Går igenom en lista med pygame events och om ett sådant är av rätt knapptryck skickar ett kommando till roboten
        """
        for event in events:
            if event.type is KEYUP:
                if event.key == K_w: self.rfClient.send(FORWARDS)
                elif event.key == K_a: self.rfClient.send(LEFT)
                elif event.key == K_s: self.rfClient.send(BACKWARDS)
                elif event.key == K_d: self.rfClient.send(RIGHT)
                elif event.key == K_SPACE: self.rfClient.send(STOP)

    def update_data(self):
        """
         Updaterar datan från roboten. Skickar en förfrågan om data från roboten och lyssnar efter ett svar som sedan
         sparas. Metoden frågar och tar bara emot ett datavärde från roboten, för att ta emot alla datavärden från
         roboten måste man kalla på metoden RETRIEVABLE_DATA+1(11 just nu) gånger.
        """
        address = ADDRESSES[self.dataAddress]
        self.rfClient.send(bytes([self.dataAddress]))
        recievedData = self.rfClient.receive()
        if len(recievedData) is not 0:
            if address is "Liv":
                if int(recievedData[0]) is 7:
                    self.data[address] = 3
                elif int(recievedData[0]) is 3:
                    self.data[address] = 2
                elif int(recievedData[0]) is 1:
                    self.data[address] = 1
                else:
                    self.data[address] = 0
            else:
                self.data[address] = int(recievedData[0])
        self.dataAddress += 1
        if self.dataAddress > RETRIEVABLE_DATA:
            self.dataAddress = 0


