import zmq
import argparse
import pygame
import orwell.proxy_simulator.tanks as tanks
import orwell.proxy_simulator.communications as communications


class Quitter(communications.BaseEventHandler):
    def _key_down(self, key):
        if (key == pygame.K_ESCAPE):
            import sys
            sys.exit(0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", dest="port", help="The port to publish to.",
                        default=5556, type=int)
    arguments = parser.parse_args()
    port = arguments.port
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://127.0.0.1:%i" % port)
    messengers = []
    descriptor = tanks.TankDescriptor(0)
    #robot = tanks.Tank(descriptor)
    robot_event_handler = communications.TankEventHandler(descriptor)
    messengers.append(robot_event_handler)
    messengers.append(Quitter())
    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption('Message shooter')
    pygame.mouse.set_visible(False)
    clock = pygame.time.Clock()
    previous_str = ""
    while (True):
        events = pygame.event.get()
        for messenger in messengers:
            messenger.handle_events(events)
            for message in messenger.get_messages():
                new_str = str(message)
                if (previous_str != new_str):
                    print "send message:", new_str
                    previous_str = new_str
                socket.send(message)
        #clock.tick(1 / 0.05)
        clock.tick(40)


if ('__main__' == __name__):
    main()
