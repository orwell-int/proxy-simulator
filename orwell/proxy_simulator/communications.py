import pygame
import orwell.proxy_simulator.version1_pb2 as pb_messages


class BaseEventHandler(object):
    """
    Base class which defines the default behaviour of the different
    event handler.
    """
    def handle_events(self, events):
        """
        Dispatch the events between #_key_down (key has been pressed),
        #_key_up (key has been released), and #_any_event (the rest).
        Events are processed in order.
        `events`: the events to handle.
        """
        for e in events:
            if (e.type == pygame.KEYDOWN):
                self._key_down(e.key)
            elif (e.type == pygame.KEYUP):
                self._key_up(e.key)
            else:
                self._any_event(e)

    def _key_down(self, key):
        """
        Override to define what happens when a given key is pressed.
        `key`: the key that has just been pressed.
        """
        pass

    def _key_up(self, key):
        """
        Override to define what happens when a given key is released.
        `key`: the key that has just been released.
        """
        pass

    def _any_event(self, event):
        """
        Override to define what happens for an event that does not match
        a key pressed or released.
        `event`: the full description of the event.
        """
        pass

    def get_messages(self):
        """
        Returns a list of messages (in reaction to the events handled).
        """
        return []


class TankEventHandler(BaseEventHandler):
    def __init__(self, robot_descriptor):
        self._robot_descriptor = robot_descriptor
        self._left = 0
        self._right = 0

    def _key_down(self, key):
        if (key == pygame.K_q):
            self._left = 1
        elif (key == pygame.K_a):
            self._left = -1
        elif (key == pygame.K_e):
            self._right = 1
        elif (key == pygame.K_d):
            self._right = -1

    def _key_up(self, key):
        if (key in (pygame.K_q, pygame.K_a)):
            self._left = 0.0
        elif (key in (pygame.K_e, pygame.K_d)):
            self._right = 0.0

    def get_messages(self):
        return [self._robot_descriptor.get_move_message(
            self._left, self._right)]


class Broadcaster(object):
    """
    Sends messages to all listeners registered.
    """
    def __init__(self):
        self._listeners = []
        self._messages = []

    def register_listener(self, listener):
        """
        Register a listener if not registered allready.
        `listener`: the listener to add.
        """
        if (listener not in self._listeners):
            self._listeners.append(listener)

    def queue(self, messages):
        """
        Queue some messages to be sent through the broadcaster. The messages
        are stored until #broadcast is called.
        `messages`: a list of messages to send.
        """
        self._messages += messages

    def broadcast(self):
        """
        Broadcast all the messages stored to all the listeners.
        Messages are sent in the order they where given.
        """
        while (self._messages):
            message = self._messages.pop(0)
            wrapper_msg = pb_messages.base_message()
            wrapper_msg.ParseFromString(message)
            for listener in self._listeners:
                listener.handle_message(wrapper_msg)
