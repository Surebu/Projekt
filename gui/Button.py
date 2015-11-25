from misc import *


class Button:
    def __init__(self, coordinates, dimensions, text, main_colour, rim_colour, text_colour, function, obj):
        self.main_colour = main_colour
        self.rim_colour = rim_colour
        self.length = dimensions[0]
        self.height = dimensions[1]
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.rect = pygame.Rect(self.x, self.y, self.length, self.height)
        self.obj = obj
        if function is None:
            self.function = no_function
        else:
            self.function = function

        font = SMALL_FONT
        self.text = font.render(text, 1, text_colour)

    def draw(self, screen):
        pygame.draw.rect(screen, self.main_colour, (self.x, self.y, self.length, self.height), 0)        # bakgrund för knappen
        pygame.draw.rect(screen, self.rim_colour, (self.x, self.y, self.length, self.height), 5)    # kant
        screen.blit(self.text, ((self.x + self.length / 2) - self.text.get_width() / 2,
                                (self.y + self.height / 2) - self.text.get_height() / 2))  # textmagi

    def contains_coordinate(self, coordinate):
        if self.rect.topleft[1] < coordinate[1] < self.rect.bottomright[1] \
                and self.rect.topleft[0] < coordinate[0] < self.rect.bottomright[0]:
            return True
        return False
