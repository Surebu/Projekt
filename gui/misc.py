"""
En samling av konstanter och hjälpfunktioner/klasser som används på flera ställen i programmet.
"""
import pygame
from pygame.locals import *



def no_function(*args):
    print("No function assigned to button or textbox")

## Kommandon
FORWARDS = b'\xff'
LEFT = b'\x0f'
RIGHT = b'\xf0'
BACKWARDS = b'\xcc'
STOP = b'\x00'

CHANGE_CONTROL_MODE = b'\xaa'

## Adresser till och i roboten och hjälpkonstanter
SUREBU1_MACADDR = '00:06:66:03:16:FC'
RETRIEVABLE_DATA = 11
ADDRESSES = [
    "IR-sensor 1",
    "IR-sensor 2",
    "IR-sensor 3",
    "IR-sensor 4",
    "Tejpsensor 1",
    "Tejpsensor 2",
    "Tejpsensor 3",
    "Tejpsensor 4",
    "Avståndssensor",
    "Träffdetektor",
    "Liv",
    "Kontroll läge"
]
# Sensorkommandon
SENSORS = [
    b'\x01',    #testsensor 1
    b'\x02',    #testsensor 2
    b'\x03',    #testsensor 3
    b'\x04',    #testsensor 4
    b'\x05',    #testsensor 5
    b'\x06',    #testsensor 6
]
## Färger

MAIN_BACKGROUND = (246, 166, 170)
RIGHT_BACKGROUND = (203, 102, 132)
BUTTON_MAIN = (225, 92, 192)
BUTTON_RIM = (132, 240, 14)
GREEN = (132, 240, 14)
BLUE = (0, 128, 255)
RED = (255, 50, 0)
BLACK = (0, 0, 0)
GREY = (150, 150, 150)

## Fonts

pygame.font.init()
FONT = pygame.font.SysFont("Calibri", 20)
SMALL_FONT = pygame.font.SysFont("Calibri", 14)
UNDERLINED_FONT = pygame.font.SysFont("Calibri", 14)
UNDERLINED_FONT.set_underline(True)


class Dims:
    """
    Bara en container-klass för att spara relationer mellan objekt i fönstret
    """
    def __init__(self, window_dimensions):
        self.totalWidth = window_dimensions[0]
        self.totalHeight = window_dimensions[1]

        self.rightPanelWidth = self.totalWidth / 2.5
        self.rightPanelStart = self.totalWidth - self.rightPanelWidth

        self.widthPadding = self.totalWidth / 16
        self.heightPadding = self.totalHeight / 24

        self.buttonWidth = self.rightPanelWidth - 4*self.heightPadding
        self.buttonHeight = self.totalHeight / 6
