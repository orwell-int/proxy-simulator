import orwell.proxy_simulator.tanks as tanks
import orwell.proxy_simulator.world as world
import orwell.proxy_simulator.communications as communications


def main():
    broadcaster = communications.LocalEventDispatcher()
    test_world = world.World(broadcaster, draw_helpers=True)
    descriptor = tanks.TankDescriptor(0)
    robot = tanks.Tank(descriptor)
    robot_event_handler = communications.TankEventHandler(descriptor)
    test_world.add(robot)
    broadcaster.register_event_handler(robot_event_handler)
    test_world.camera_giver = robot
    broadcaster.register_listener(robot)
    broadcaster.register_listener(test_world)
    test_world.run()

if (__name__ == '__main__'):
    main()
