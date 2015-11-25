from misc import *

LIFE = pygame.image.load("life.jpg")
LIFERECT = LIFE.get_rect()


class Surebu:
    def __init__(self):
        self.irSensors = [0, 200, 150, 12]
        self.tapeSensors = [False, False, False, False]
        self.distanceSensor = 0
        self.hitDetector = False
        self.life = 3
        self.controlMode = "CONTROL"

    def change_control_mode(self):
        if self.controlMode is "CONTROL":
            self.controlMode = "AUTONOMOUS"
        else:
            self.controlMode = "CONTROL"

    def draw(self, screen, dims):
        self.__draw_list(screen, self.irSensors, "IR-sensor ", (dims.widthPadding / 4, dims.heightPadding))
        self.__draw_list(screen, self.tapeSensors, "Tejpsensor ", (dims.buttonWidth, dims.heightPadding))

        text = FONT.render("AVSTÅND: " + str(self.distanceSensor), 5, BLACK)
        screen.blit(text, (dims.widthPadding / 4, dims.totalHeight / 2.2))

        text = FONT.render("TRÄFFDETEKTOR: " + str(self.hitDetector), 5, BLACK)
        screen.blit(text, (dims.widthPadding / 4, dims.totalHeight / 2.2 + dims.heightPadding))

        text = FONT.render("Liv:", 5, BLACK)
        screen.blit(text, (dims.widthPadding / 4, dims.totalHeight / 2.2 + 3*dims.heightPadding))
        for i in range(self.life):
            screen.blit(LIFE, (dims.widthPadding / 4 + i * (LIFERECT.width + dims.widthPadding / 4),
                               dims.totalHeight / 2.2 + 4*dims.heightPadding))

    def __draw_list(self, screen, list, name, coordinates):
        j = 0
        for i in list:
            text = FONT.render(name + str(j + 1) + ": " + str(i), 5, BLACK)
            screen.blit(text, (coordinates[0], coordinates[1] + 30*j))
            j += 1

