import pygame
import sys, os, traceback

from Button import *
import eztext
from pygame import *
from time import *

fireflyBaddr = '00:06:66:03:16:FC'
# Center the screen
if sys.platform in ["win32", "win64"]: os.environ["SDL_VIDEO_CENTERED"] = "1"

"""
#RfComm
firefly = RfCommClient(fireflyBaddr,1)
while not firefly.connect():
    pass
"""


def print_hello():
    print("hello")


##Färger

BLUE = (0, 128, 255)
RED = (255, 50, 0)
BLACK = (0, 0, 0)
GREY = (150, 150, 150)


class GUI:
    def __init__(self, window_dimensions, caption):
        pygame.display.init()
        pygame.font.init()
        pygame.display.set_caption(caption)
        self.done = False
        self.screen = pygame.display.set_mode(window_dimensions)
        self.buttons = []
        self.textbox = eztext.Input(maxlength=10, color=RED, prompt='type here: ')

    @staticmethod
    def blank_icon():
        icon = pygame.Surface((1, 1))
        icon.set_alpha(0)
        pygame.display.set_icon(icon)

    def check_buttons(self, coordinate):
        for button in self.buttons:
            if button.contains_coordinate(coordinate):
                button.function()

    def paint(self):
        self.screen.fill(GREY)
        for button in self.buttons:
            button.draw(self.screen)
        self.textbox.draw(self.screen)


    def run(self):

        while not gui.done:
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    gui.done = True
                if event.type == MOUSEBUTTONDOWN:
                    self.check_buttons(pygame.mouse.get_pos())
            self.textbox.update(events)
            gui.paint()
            pygame.display.flip()


gui = GUI([640, 480], "teos test")
gui.blank_icon()
gui.buttons.append(Button((20, 20), (300, 100), "hej", BLUE, RED, print_hello))
gui.buttons.append(Button((20, 140), (300, 100), "hej", BLUE, RED, print_hello))
gui.run()
