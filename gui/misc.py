import pygame


def no_function(*args):
    print("No function assigned to button or textbox")

## Kommandon
FORWARDS = b'\xff'
LEFT = b'\x0f'
RIGHT = b'\xf0'
BACKWARDS = b'\xcc'
STOP = b'\x00'

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
    def __init__(self, window_dimensions):
        self.totalWidth = window_dimensions[0]
        self.totalHeight = window_dimensions[1]

        self.rightPanelWidth = self.totalWidth / 2.5
        self.rightPanelStart = self.totalWidth - self.rightPanelWidth

        self.widthPadding = self.totalWidth / 16
        self.heightPadding = self.totalHeight / 24

        self.buttonWidth = self.rightPanelWidth - 4*self.heightPadding
        self.buttonHeight = self.totalHeight / 6
