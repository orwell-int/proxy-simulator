import zmq
import argparse
import pygame
import orwell.proxy_simulator.tanks as tanks
import orwell.proxy_simulator.world as world
import orwell.proxy_simulator.communications as communications


class Quitter(communications.BaseEventHandler):
    def _key_down(self, key):
        if (key == pygame.K_ESCAPE):
            import sys
            sys.exit(0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", dest="port", help="The port to read from.",
                        default=5556, type=int)
    arguments = parser.parse_args()
    port = arguments.port
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://*:%i" % port)
    socket.setsockopt(zmq.SUBSCRIBE, "")

    broadcaster = communications.CombinedDispatcher(socket)
    test_world = world.World(broadcaster, draw_helpers=True)
    descriptor = tanks.TankDescriptor(0)
    robot = tanks.Tank(descriptor)
    robot_event_handler = communications.TankEventHandler(descriptor)
    test_world.add(robot)
    broadcaster.register_event_handler(Quitter())
    broadcaster.register_event_handler(robot_event_handler)
    test_world.camera_giver = robot
    broadcaster.register_listener(robot)
    broadcaster.register_listener(test_world)
    test_world.run()

    if (False):
        messengers = []
        messengers.append(Quitter())
        pygame.init()
        screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption('Message receiver')
        pygame.mouse.set_visible(False)
        clock = pygame.time.Clock()
        previous_str = ""
        has_received = False
        is_receiving = False

        while ((not has_received) or (has_received) and (is_receiving)):
            events = pygame.event.get()
            for messenger in messengers:
                messenger.handle_events(events)
            try:
                new_str = socket.recv(flags=zmq.DONTWAIT)
                has_received = True
                is_receiving = True
                if (previous_str != new_str):
                    print "receive message:", new_str
                    previous_str = new_str
            except Exception as e:
                print e
                is_receiving = False
            clock.tick(1 / 0.05)


if ('__main__' == __name__):
    main()
