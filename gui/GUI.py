import sys, os, traceback
from Button import *
from surebu import *
import eztext


# Center the screen
if sys.platform in ["win32", "win64"]: os.environ["SDL_VIDEO_CENTERED"] = "1"


class GUI:
    """
    Viewer-klass för för Surebu-objektet. Ritar ut ett Surebu-objekt i ett pygame-fönster
    """
    def __init__(self, window_dimensions, caption):

        pygame.display.init()

        pygame.display.set_caption(caption)
        self.done = False
        self.screen = pygame.display.set_mode(window_dimensions)
        self.buttons = []
        self.surebu = Surebu()

        self.dims = Dims(window_dimensions)
        self.MAIN_BACKGROUND_SURFACE = pygame.Surface((self.dims.totalWidth, self.dims.totalHeight))
        self.init_background()

        self.btaddr = eztext.Input(x=self.dims.rightPanelStart + 2 * self.dims.heightPadding,
                                   y=self.dims.heightPadding, font=SMALL_FONT, maxlength=17, prompt="MACADDR:",
                                   function=FakeRFCommClient.set_host_address, obj=self.surebu.rfClient)

        self.buttons.append(Button((self.dims.rightPanelStart + 2 * self.dims.heightPadding, 2 * self.dims.heightPadding),
                                   (self.dims.buttonWidth, self.dims.buttonHeight), "BLUETOOTH CONNECT",
                                   BUTTON_MAIN, BUTTON_RIM, BLACK, Surebu.connect_disconnect_button, self.surebu))

        self.buttons.append(Button((self.dims.rightPanelStart + 2 * self.dims.heightPadding, 8 * self.dims.heightPadding),
                                   (self.dims.buttonWidth, self.dims.buttonHeight), "AUTONOM/CONTROLED",
                                   BUTTON_MAIN, BUTTON_RIM, BLACK, Surebu.change_control_mode, self.surebu))

        self.btaddr.value = self.surebu.rfClient.host

    def init_background(self):
        """
        Hjälpmetod för att färglägga de två fälten på bakgrunden
        """
        self.MAIN_BACKGROUND_SURFACE.fill(MAIN_BACKGROUND)
        right_background_surface = pygame.Surface((self.dims.rightPanelWidth, self.dims.totalHeight))
        right_background_surface.fill(RIGHT_BACKGROUND)
        self.MAIN_BACKGROUND_SURFACE.blit(right_background_surface, (self.screen.get_width() - right_background_surface.get_width(), 0))

    @staticmethod
    def blank_icon():
        """
        Tar bort stardard ikonen för ett pygame-fönster och sätter den blank istället
        """
        icon = pygame.Surface((1, 1))
        icon.set_alpha(0)
        pygame.display.set_icon(icon)

    def check_buttons(self, coordinate):
        """
        Kollar om koordinaterna ligger innanför någon av knapparnas kant, i sådana fall görs knappens funktion
        """
        for button in self.buttons:
            if button.contains_coordinate(coordinate):
                if button.obj is None:
                    button.function()
                else:
                    button.function(button.obj)

    def paint(self):
        """
        Ritar ut GUI:ts alla delar i fönstret.
        """
        self.screen.blit(self.MAIN_BACKGROUND_SURFACE, (0, 0))
        for button in self.buttons:
            button.draw(self.screen)
        self.surebu.draw(self.screen, self.dims)
        self.btaddr.draw(self.screen)
"""
    def run(self):
        while not self.done:
            pygame.time.Clock().tick(30)
            events = pygame.event.get()
            if self.surebu.rfClient.status is not "CONNECTED":
                self.btaddr.update(events)
            elif self.surebu.controlMode is "CONTROL":
                self.surebu.control(events)
            for event in events:
                if event.type == pygame.QUIT:
                    self.done = True
                if event.type is MOUSEBUTTONDOWN and event.button is 1:
                    self.check_buttons(pygame.mouse.get_pos())
            if self.surebu.rfClient.status is "CONNECTED":
                self.surebu.update_data()
            self.paint()
            pygame.display.flip()
"""

