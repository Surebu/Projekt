import sys, os, traceback
from Button import *
from surebu import *
from SurebuControl import *
import eztext
from pygame import *
from time import *
from RFCommClient import *

fireflyBaddr = '00:06:66:03:16:FC'

# Center the screen
if sys.platform in ["win32", "win64"]: os.environ["SDL_VIDEO_CENTERED"] = "1"


class GUI:
    def __init__(self, window_dimensions, caption):

        pygame.display.init()

        pygame.display.set_caption(caption)
        self.done = False
        self.screen = pygame.display.set_mode(window_dimensions)
        self.buttons = []
        self.surebu = Surebu()
        self.rfComm = FakeRFCommClient("hej", 1)

        self.dims = Dims(window_dimensions)

        self.MAIN_BACKGROUND_SURFACE = pygame.Surface(window_dimensions)
        self.MAIN_BACKGROUND_SURFACE.fill(MAIN_BACKGROUND)
        self.RIGHT_BACKGROUND_SURFACE = pygame.Surface((self.dims.rightPanelWidth, self.dims.totalHeight))
        self.RIGHT_BACKGROUND_SURFACE.fill(RIGHT_BACKGROUND)
        self.MAIN_BACKGROUND_SURFACE.blit(self.RIGHT_BACKGROUND_SURFACE, (self.screen.get_width() - self.RIGHT_BACKGROUND_SURFACE.get_width(), 0))

        self.btaddr = eztext.Input(x=self.dims.rightPanelStart + 2 * self.dims.heightPadding,
                                   y=self.dims.heightPadding, font=SMALL_FONT, maxlength=17, prompt="MACADDR:",
                                   function=FakeRFCommClient.set_host_address, obj=self.rfComm)

        self.buttons.append(Button((self.dims.rightPanelStart + 2 * self.dims.heightPadding, 2 * self.dims.heightPadding),
                                   (self.dims.buttonWidth, self.dims.buttonHeight), "BLUETOOTH CONNECT",
                                   BUTTON_MAIN, BUTTON_RIM, BLACK, FakeRFCommClient.connect_disconnect_button, self.rfComm))
        self.buttons.append(Button((self.dims.rightPanelStart + 2 * self.dims.heightPadding, 8 * self.dims.heightPadding),
                                   (self.dims.buttonWidth, self.dims.buttonHeight), "AUTONOM/CONTROLED",
                                   BUTTON_MAIN, BUTTON_RIM, BLACK, Surebu.change_control_mode, self.surebu))

    @staticmethod
    def blank_icon():
        icon = pygame.Surface((1, 1))
        icon.set_alpha(0)
        pygame.display.set_icon(icon)

    def check_buttons(self, coordinate):
        for button in self.buttons:
            if button.contains_coordinate(coordinate):
                if button.obj is None:
                    button.function()
                else:
                    button.function(button.obj)

    def draw_status(self):
        text1 = SMALL_FONT.render("STATUS: ", 5, BLACK)

        if self.surebu.controlMode is "CONTROL":
            text2 = UNDERLINED_FONT.render(self.surebu.controlMode, 5, GREEN)
        else:
            text2 = UNDERLINED_FONT.render(self.surebu.controlMode, 5, RED)

        if self.rfComm.status is "CONNECTED":
            text3 = UNDERLINED_FONT.render(self.rfComm.status, 5, GREEN)
        else:
            text3 = UNDERLINED_FONT.render(self.rfComm.status, 5, RED)

        self.screen.blit(text1, (self.dims.rightPanelStart + self.dims.widthPadding,
                                 self.dims.buttonHeight + self.dims.heightPadding*2.5))
        self.screen.blit(text3, (self.dims.rightPanelStart + 2.2*self.dims.widthPadding,
                                 self.dims.buttonHeight + self.dims.heightPadding*2.5))
        self.screen.blit(text1, (self.dims.rightPanelStart + self.dims.widthPadding,
                                 2*self.dims.buttonHeight + self.dims.heightPadding*4.5))
        self.screen.blit(text2, (self.dims.rightPanelStart + 2.2*self.dims.widthPadding,
                                 2*self.dims.buttonHeight + self.dims.heightPadding*4.5))

    def paint(self):
        self.screen.blit(self.MAIN_BACKGROUND_SURFACE, (0, 0))
        for button in self.buttons:
            button.draw(self.screen)
        self.surebu.draw(self.screen, self.dims)
        self.draw_status()
        self.btaddr.draw(self.screen)

    def run(self):
        while not gui.done:
            events = pygame.event.get()
            if self.rfComm.status is not "CONNECTED":
                self.btaddr.update(events)
            elif self.surebu.controlMode is "CONTROL":
                control(events,self.rfComm)
            for event in events:
                if event.type == pygame.QUIT:
                    gui.done = True
                if event.type is MOUSEBUTTONDOWN and event.button is 1:
                    self.check_buttons(pygame.mouse.get_pos())

            gui.paint()
            pygame.display.flip()


gui = GUI([640, 480], "teos test")
gui.blank_icon()
gui.run()
