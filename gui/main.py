from GUI import *


## Koden körs härifrån, mest för enkelhetens skull.
gui = GUI([640, 480], "S.L.A.V.E.")
gui.blank_icon()

while not gui.done:
    pygame.time.Clock().tick(200)
    events = pygame.event.get()
    if gui.surebu.rfClient.status is not "CONNECTED":
        gui.btaddr.update(events)
    for event in events:
        if event.type == pygame.QUIT:
            gui.done = True
        if event.type is MOUSEBUTTONDOWN and event.button is 1:
            gui.check_buttons(pygame.mouse.get_pos())
    if gui.surebu.rfClient.status is "CONNECTED":
        gui.surebu.update_data()
    gui.paint()
    pygame.display.flip()
