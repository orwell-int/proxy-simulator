import pygame
import zmq
import sys
import time
import datetime
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
        self._fire1 = False
        self._fire2 = False

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
        return [self._robot_descriptor.get_input_message(
            self._left, self._right, self._fire1, self._fire2)]


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
            recipient, _, typed_payload = message.partition(' ')
            message_type, _, payload = typed_payload.partition(' ')
            for listener in self._listeners:
                listener.handle_message(recipient, message_type, payload)


class LocalEventDispatcher(Broadcaster):
    def __init__(self):
        super(LocalEventDispatcher, self).__init__()
        self._event_handlers = []

    def register_event_handler(self, event_handler):
        if (event_handler not in self._event_handlers):
            self._event_handlers.append(event_handler)

    def step(self):
        events = pygame.event.get()
        for event_handler in self._event_handlers:
            event_handler.handle_events(events)
            self.queue(event_handler.get_messages())
        self.broadcast()


class CombinedDispatcher(Broadcaster):
    def __init__(self, receiver_socket):
        super(CombinedDispatcher, self).__init__()
        self._event_handlers = []
        self._receiver_socket = receiver_socket
        self._string = None

    def register_event_handler(self, event_handler):
        if (event_handler not in self._event_handlers):
            self._event_handlers.append(event_handler)

    def step(self):
        events = pygame.event.get()
        for event_handler in self._event_handlers:
            event_handler.handle_events(events)
            self.queue(event_handler.get_messages())
        start_time = datetime.datetime.now()
        last_limit = start_time + datetime.timedelta(milliseconds=10)
        string = None
        #for _ in range(10):
        while (True):
            if (datetime.datetime.now() > last_limit):
                break
            try:
                string = self._receiver_socket.recv(flags=zmq.DONTWAIT)
                if ((self._string is None) or (string != self._string)):
                    print "received message:" + string
                    self._string = string
                else:
                    sys.stdout.write('.')
            except zmq.ZMQError as e:
                #print e
                #time.sleep(0.001)
                pass
        if (self._string is not None):
            self.queue([self._string])
        self.broadcast()
