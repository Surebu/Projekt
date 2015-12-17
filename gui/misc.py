"""
En samling av konstanter och hjälpfunktioner/klasser som används på flera ställen i programmet.
"""
import pygame
from pygame.locals import *



def no_function(*args):
    print("No function assigned to button or textbox")

## Adresser till och i roboten och hjälpkonstanter
SUREBU1_MACADDR = '00:06:66:03:16:FC'
RETRIEVABLE_DATA = 13
ADDRESSES = [
    "IR-sensor 1 V",
    "TargetingM flaggor1",
    "IR-sensor 2 F",
    "IR-sensor 3 H",
    "Tejpsensor 1 VF",
    "TargetingM flaggor3",
    "TargetingM flaggor2",
    "Tejpsensor 2 HF",
    "Avståndssensor",
    "Träffdetektor",
    "Tapevalues",
    "Liv",
    "Kontrolläge",
    "Move"
]

DEBUG_ADDRESSES = [
    "TargetingM flaggor1",
    "Tejpsensor 2 VB",
    "TargetingM flaggor3",
    "TargetingM flaggor2",
    "Tapevalues",
    "Kontrolläge",
    "Move"
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
