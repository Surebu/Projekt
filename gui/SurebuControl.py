from RFCommClient import *
from pygame.locals import *
from misc import *


def control(events, rfClient):
    for event in events:
        if event.type is KEYUP:
            if event.key == K_w: rfClient.send(FORWARDS)
            elif event.key == K_a: rfClient.send(LEFT)
            elif event.key == K_s: rfClient.send(BACKWARDS)
            elif event.key == K_d: rfClient.send(RIGHT)
            elif event.key == K_SPACE: rfClient.send(STOP)
