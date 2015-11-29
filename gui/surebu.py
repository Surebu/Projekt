from misc import *
from RFCommClient import *

LIFE = pygame.image.load("life.png")
LIFE_RECT = LIFE.get_rect()


class Surebu:
    """
    Container som håller alla värden från roboten och uppkopplingen till den. Är också en kontroller i den mån att den
    skickar kommandon och förfrågningar till roboten över uppkopplingen.
    """
    def __init__(self):
        self.data = {
            "IR-sensor 1": 0,
            "IR-sensor 2": 0,
            "IR-sensor 3": 0,
            "IR-sensor 4": 0,
            "Tejpsensor 1": 0,
            "Tejpsensor 2": 0,
            "Tejpsensor 3": 0,
            "Tejpsensor 4": 0,
            "Avståndssensor": 0,
            "Träffdetektor": 0,
            "Liv": 3,
            "Kontrolläge":0
        }
        self.dataAddress = 0
        self.rfClient = FakeRFCommClient(SUREBU1_MACADDR, 1)

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
            while not self.rfClient.connect():
                pass
            self.rfClient.send(ADDRESSES.index("Kontrolläge"))
            self.data["Kontrolläge"] = self.rfClient.receive()
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
                text = FONT.render(address + ": ", 5, BLACK)
                screen.blit(text, (dims.widthPadding / 4, dims.totalHeight / 2.2 + 3*dims.heightPadding))
                for j in range(self.data[address]):
                    screen.blit(LIFE, (dims.widthPadding / 4 + j * (LIFE_RECT.width + dims.widthPadding / 4),
                                       dims.totalHeight / 2.2 + 4*dims.heightPadding))
            i += 1
        self.__draw_status(screen, dims)

    def __draw_status(self, screen, dims):
        """
        Hjälpmetod för att rita ut statusen för uppkopplingen till roboten och i vilket kontrolllägen roboten är i.
        """
        status = SMALL_FONT.render("STATUS: ", 5, BLACK)

        if self.rfClient.status is "CONNECTED":
            connection_satus = UNDERLINED_FONT.render(self.rfClient.status, 5, GREEN)
            if self.data["Kontrolläge"] is 1:
                control_status = UNDERLINED_FONT.render("CONTROL", 5, GREEN)
            else:
                control_status = UNDERLINED_FONT.render("AUTONOMOUS", 5, RED)
        else:
            connection_satus = UNDERLINED_FONT.render(self.rfClient.status, 5, RED)
            control_status = SMALL_FONT.render("N/a", 5, RED)

        screen.blit(status, (dims.rightPanelStart + dims.widthPadding,
                            dims.buttonHeight + dims.heightPadding*2.5))
        screen.blit(connection_satus, (dims.rightPanelStart + 2.2*dims.widthPadding,
                            dims.buttonHeight + dims.heightPadding*2.5))
        screen.blit(status, (dims.rightPanelStart + dims.widthPadding,
                            2*dims.buttonHeight + dims.heightPadding*4.5))
        screen.blit(control_status, (dims.rightPanelStart + 2.2*dims.widthPadding,
                            2*dims.buttonHeight + dims.heightPadding*4.5))

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
        self.rfClient.send(address)
        self.data[address] = self.rfClient.receive()
        self.dataAddress += 1
        if self.dataAddress > RETRIEVABLE_DATA:
            self.dataAddress = 0


