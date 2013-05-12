import ode
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import orwell.proxy_simulator.version1_pb2 as pb_messages


class Tank(object):

    def __init__(self, robot_descriptor):
        self._velocity_left = 0
        self._velocity_right = 0
        self._left_wheel_body = None
        self._right_wheel_body = None
        self._robot_descriptor = robot_descriptor

    def create_objects(self, world):
        print 'chassis'
        # chassis
        density = 10
        lx, ly, lz = (8, 0.5, 8)
        # Create body
        body = ode.Body(world.world)
        mass = ode.Mass()
        mass.setBox(density, lx, ly, lz)
        body.setMass(mass)

        # Set parameters for drawing the body
        body.shape = "box"
        body.boxsize = (lx, ly, lz)

        # Create a box geom for collision detection
        geom = ode.GeomBox(world.space, lengths=body.boxsize)
        geom.setBody(body)
        #body.setPosition((0, 3, 0))
        world.add_body(body)
        world.add_geom(geom)

        density = 1
        print 'left wheel'
        # left wheel
        radius = 1
        height = 0.2
        px, py, pz = (4, 0, -4)
        self._left_wheel_body = ode.Body(world.world)
        wheel_mass = ode.Mass()
        #wheel_mass.setSphere(density, radius)
        wheel_mass.setCylinder(density, 1, radius, height)
        self._left_wheel_body.setMass(wheel_mass)
        #left_wheel_geom = ode.GeomSphere(world.space, radius=radius)
        left_wheel_geom = ode.GeomCylinder(world.space, radius=radius,
                                           length=height)
        left_wheel_geom.setBody(self._left_wheel_body)
        #self._left_wheel_body.setPosition((px, py, pz))
        self._left_wheel_body.setRotation((0, 0, 1,
                                           0, 1, 0,
                                           -1, 0, 0))
        self._left_wheel_body.setPosition((px - height / 2, py, pz))
        world.add_body(self._left_wheel_body)
        world.add_geom(left_wheel_geom)

        print 'right wheel'
        # right wheel
        #radius = 1
        px = -4
        self._right_wheel_body = ode.Body(world.world)
        wheel_mass = ode.Mass()
        #wheel_mass.setSphere(density, radius)
        wheel_mass.setCylinder(density, 1, radius, height)
        self._right_wheel_body.setMass(wheel_mass)
        #right_wheel_geom = ode.GeomSphere(world.space, radius=radius)
        right_wheel_geom = ode.GeomCylinder(world.space, radius=radius,
                                            length=height)
        right_wheel_geom.setBody(self._right_wheel_body)
        #self._right_wheel_body.setPosition((px, py, pz))
        self._right_wheel_body.setRotation((0, 0, 1,
                                            0, 1, 0,
                                            -1, 0, 0))
        self._right_wheel_body.setPosition((px - height / 2, py, pz))
        world.add_body(self._right_wheel_body)
        world.add_geom(right_wheel_geom)

        print 'front wheel'
        # front wheel
        #radius = 1
        px, py, pz = (0, 0, 4)
        front_wheel_body = ode.Body(world.world)
        wheel_mass = ode.Mass()
        wheel_mass.setSphere(density, radius)
        front_wheel_body.setMass(wheel_mass)
        front_wheel_geom = ode.GeomSphere(world.space, radius=radius)
        front_wheel_geom.setBody(front_wheel_body)
        front_wheel_body.setPosition((px, py, pz))
        world.add_body(front_wheel_body)
        world.add_geom(front_wheel_geom)

        #self._left_wheel_joint = ode.Hinge2Joint(world.world)
        self._left_wheel_joint = ode.HingeJoint(world.world)
        self._left_wheel_joint.attach(body, self._left_wheel_body)
        self._left_wheel_joint.setAnchor(self._left_wheel_body.getPosition())
        self._left_wheel_joint.setAxis((-1, 0, 0))
        #self._left_wheel_joint.setAxis1((0, 1, 0))
        #self._left_wheel_joint.setAxis2((1, 0, 0))
        self._left_wheel_joint.setParam(ode.ParamFMax, 500000)
        #self._left_wheel_joint.setParam(ode.ParamLoStop, 0)
        #self._left_wheel_joint.setParam(ode.ParamHiStop, 0)
        #self._left_wheel_joint.setParam(ode.ParamFMax2, 0.1)
        #self._left_wheel_joint.setParam(ode.ParamSuspensionERP, 0.2)
        #self._left_wheel_joint.setParam(ode.ParamSuspensionCFM, 0.1)

        #self._right_wheel_joint = ode.Hinge2Joint(world.world)
        self._right_wheel_joint = ode.HingeJoint(world.world)
        self._right_wheel_joint.attach(body, self._right_wheel_body)
        self._right_wheel_joint.setAnchor(self._right_wheel_body.getPosition())
        self._right_wheel_joint.setAxis((-1, 0, 0))
        #self._right_wheel_joint.setAxis1((0, 1, 0))
        #self._right_wheel_joint.setAxis2((1, 0, 0))
        self._right_wheel_joint.setParam(ode.ParamFMax, 500000)
        #self._right_wheel_joint.setParam(ode.ParamLoStop, 0)
        #self._right_wheel_joint.setParam(ode.ParamHiStop, 0)
        #self._right_wheel_joint.setParam(ode.ParamFMax2, 0.1)
        #self._right_wheel_joint.setParam(ode.ParamSuspensionERP, 0.2)
        #self._right_wheel_joint.setParam(ode.ParamSuspensionCFM, 0.1)

        front_wheel_joint = ode.BallJoint(world.world)
        front_wheel_joint.attach(body, front_wheel_body)
        front_wheel_joint.setAnchor(front_wheel_body.getPosition())
        front_wheel_joint.setParam(ode.ParamFMax, 5000)

    @property
    def velocity_left(self):
        return self._velocity_left

    @velocity_left.setter
    def velocity_left(self, value):
        self._velocity_left = value

    @property
    def velocity_right(self):
        return self._velocity_right

    @velocity_right.setter
    def velocity_right(self, value):
        self._velocity_right = value

    def update(self):
        self._left_wheel_joint.setParam(ode.ParamVel, self._velocity_left)
        self._right_wheel_joint.setParam(ode.ParamVel, self._velocity_right)

    def handle_message(self, wrapper_msg):
        if (self._robot_descriptor.recipient == wrapper_msg.recipient):
            left, right = \
                    self._robot_descriptor.get_movement(wrapper_msg)
            self.velocity_left = 10 * left
            self.velocity_right = 10 * right


class TankDescriptor(object):
    def __init__(self, robot_id):
        self._robot_id = robot_id
        self._recipient = "TANK_%i" % self._robot_id

    @property
    def recipient(self):
        return self._recipient

    def get_move_message(self, left, right):
        assert(-1 <= left <= 1)
        assert(-1 <= right <= 1)
        sub_msg = pb_messages.move_tank_message()
        sub_msg.left = left
        sub_msg.right = right
        wrapper_msg = pb_messages.base_message()
        wrapper_msg.message_type = "MOVE_TANK"
        wrapper_msg.serialized_message = sub_msg.SerializeToString()
        wrapper_msg.recipient = self._recipient
        return wrapper_msg.SerializeToString()

    def get_movement(self, wrapper_msg):
        assert("MOVE_TANK" == wrapper_msg.message_type)
        sub_msg = pb_messages.move_tank_message()
        sub_msg.ParseFromString(wrapper_msg.serialized_message)
        assert(-1 <= sub_msg.left <= 1)
        assert(-1 <= sub_msg.right <= 1)
        return (sub_msg.left, sub_msg.right)


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


class World(BaseEventHandler):
    cameraDistance = 10.0
    clip = 100.0
    fps = 50.0

    def __init__(self, broadcaster, resolution=(1024, 768)):
        self._resolution = resolution
        self._broadcaster = broadcaster
        self._xRot = 0.0
        self._yRot = 0.0
        self._xCoeff = 360.0 / self._resolution[0]
        self._yCoeff = 360.0 / self._resolution[1]
        self._geoms = []
        self._bodies = []

        self._init_opengl()
        self._init_ode()
        self._robots = []
        self._velocity_left = 0
        self._velocity_right = 0
        self._running = False
        self._cjoints = ode.JointGroup()
        self._event_handlers = [self]

    def _init_opengl(self):
        """
        Initialise the scene.
        """

        # Create a window
        pygame.init()
        screen = pygame.display.set_mode(self._resolution,
                                         pygame.OPENGL | pygame.DOUBLEBUF)
        pygame.display.set_caption('PyODE Vehicle Demo')
        pygame.mouse.set_visible(False)
        glutInit()

        glViewport(0, 0, self._resolution[0], self._resolution[1])
        glClearColor(0.8, 0.8, 0.9, 0)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_NORMALIZE)
        glShadeModel(GL_FLAT)

    def _init_ode(self):
        self._world = ode.World()
        self._world.setGravity((0, -9.81, 0))
        self._space = ode.Space()
        self._floor = ode.GeomPlane(self._space, (0, 1, 0), -1)

    def _extract_matrix(self, geom):
        """
        Return a 4x4 matrix (represented by a 16-element tuple) created by
        combining the geom's rotation matrix and position.
        """

        x, y, z = geom.getPosition()
        rot = geom.getRotation()
        return (rot[0], rot[3], rot[6], 0.0,
                rot[1], rot[4], rot[7], 0.0,
                rot[2], rot[5], rot[8], 0.0,
                x, y, z, 1.0)

    def _render_geom(self, geom):
        """
        Render either a ode.GeomBox or ode.GeomSphere object.
        """

        allowed = [ode.GeomBox, ode.GeomSphere, ode.GeomCylinder]
        ok = False
        for klass in allowed:
            ok = ok or isinstance(geom, klass)
        if (not ok):
            return

        glPushMatrix()
        glMultMatrixd(self._extract_matrix(geom))

        glMaterialfv(GL_FRONT, GL_SPECULAR, (0.0, 0.0, 0.0))

        if (isinstance(geom, ode.GeomBox)):
            sx, sy, sz = geom.getLengths()
            glScale(sx, sy, sz)
            glutSolidCube(1)
        elif (isinstance(geom, ode.GeomSphere)):
            r = geom.getRadius()
            glutSolidSphere(r, 20, 20)
        elif (isinstance(geom, ode.GeomCylinder)):
            r, h = geom.getParams()
            glutSolidCylinder(r, h, 20, 20)

        glPopMatrix()

    def _render_ground(self):
        """
        Renders the ground plane.
        """

        # Draw a quad at the position of the vehicle that extends to the
        # clipping planes.

        normal, d = self._floor.getParams()
        #x, y, z = self.chassis.getPosition()

        glPushMatrix()
        #glTranslate(x, 0.0, z)

        glMaterialfv(GL_FRONT, GL_SPECULAR, (0.0, 1.0, 0.0))

        glBegin(GL_QUADS)
        glColor3f(0.0, 1.0, 0.0)
        glNormal3f(*normal)
        glVertex3f(-self.clip, d, -self.clip)
        glNormal3f(*normal)
        glVertex3f(self.clip, d, -self.clip)
        glNormal3f(*normal)
        glVertex3f(self.clip, d, self.clip)
        glNormal3f(*normal)
        glVertex3f(-self.clip, d, self.clip)
        glEnd()

        glPopMatrix()

    def _set_camera(self):
        """
        Position the camera to C{self.cameraDistance} units behind the
        vehicle's current position and rotated depending on the mouse position.
        """

        aspect = float(self._resolution[0]) / float(self._resolution[1])

        x, y = pygame.mouse.get_pos()
        self._xRot = (y - self._resolution[1] / 2) * self._xCoeff
        self._yRot = (x - self._resolution[0] / 2) * self._yCoeff
        if (self._xRot < 0):
            self._xRot = 0

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-aspect, aspect, -1.0, 1.0, 1.5, self.clip)

        glLightfv(GL_LIGHT0, GL_POSITION, (-5.0, 10.0, 0, 0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))
        glEnable(GL_LIGHT0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Set the camera angle to view the vehicle
        glTranslate(0.0, 0.0, -self.cameraDistance)
        glRotate(self._xRot, 1, 0, 0)
        glRotate(self._yRot, 0, 1, 0)

        ## Set the camera so that the vehicle is drawn in the correct place.
        #x, y, z = self.chassis.getPosition()
        #glTranslate(-x, -y, -z)

    def render(self):
        """
        Render the current simulation state.
        """

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._render_ground()

        self._set_camera()
        #print "len(self._geoms) =", len(self._geoms)
        for geom in self._geoms:
            self._render_geom(geom)

        glFlush()
        pygame.display.flip()

    def _any_event(self, event):
        if (event.type == pygame.QUIT):
            self._running = False

    def _key_down(self, key):
        if (key == pygame.K_q):
            self._velocity_left = 10
        elif (key == pygame.K_a):
            self._velocity_left = -10
        elif (key == pygame.K_e):
            self._velocity_right = 10
        elif (key == pygame.K_d):
            self._velocity_right = -10
        elif (key == pygame.K_ESCAPE):
            self._running = False

    def _key_up(self, key):
        if (key in (pygame.K_q, pygame.K_a)):
            self._velocity_left = 0.0
        elif (key in (pygame.K_e, pygame.K_d)):
            self._velocity_right = 0.0

    def register_event_handler(self, event_handler):
        if (event_handler not in self._event_handlers):
            self._event_handlers.append(event_handler)

    def do_events(self):
        """
        Process any input events.
        """

        events = pygame.event.get()
        for event_handler in self._event_handlers:
            event_handler.handle_events(events)
            self._broadcaster.queue(event_handler.get_messages())
        self._broadcaster.broadcast()
        #for e in events:
            #if (e.type == pygame.QUIT):
                #self._running = False
            #elif (e.type == pygame.KEYDOWN):
                #self._key_down(e.key)
            #elif (e.type == pygame.KEYUP):
                #self._key_up(e.key)

    def handle_message(self, wrapper_msg):
        pass

    def _nearcb(self, args, geom1, geom2):
        """
        Create contact joints between colliding geoms.
        """

        body1, body2 = geom1.getBody(), geom2.getBody()
        if (body1 is None):
            body1 = ode.environment
        if (body2 is None):
            body2 = ode.environment

        if (ode.areConnected(body1, body2)):
            return

        contacts = ode.collide(geom1, geom2)

        for c in contacts:
            c.setBounce(0.2)
            c.setMu(10000)
            j = ode.ContactJoint(self.world, self._cjoints, c)
            j.attach(body1, body2)

    def run(self):
        """
        Start the demo. This method will block until the demo exits.
        """

        clock = pygame.time.Clock()
        self._running = True

        while self._running:
            self.do_events()

            for robot in self._robots:
                #robot.velocity_left = self._velocity_left
                #robot.velocity_right = self._velocity_right
                robot.update()

            self._space.collide((), self._nearcb)
            self._world.step(1 / self.fps)
            self._cjoints.empty()
            self.render()

            # Limit the FPS.
            clock.tick(self.fps)
            # The approach works if the simulation could run faster than it
            # is actally limited to which should work here because it is simple
            # enough and in any case it is a little slower, it is not very
            # important as we are only dealing with a simulator.

    def add(self, robot):
        self._robots.append(robot)
        robot.create_objects(self)

    @property
    def world(self):
        return self._world

    @property
    def space(self):
        return self._space

    def add_geom(self, geom):
        self._geoms.append(geom)

    def add_body(self, body):
        self._bodies.append(body)


def main():
    broadcaster = Broadcaster()
    world = World(broadcaster)
    descriptor = TankDescriptor(0)
    robot = Tank(descriptor)
    robot_event_handler = TankEventHandler(descriptor)
    world.add(robot)
    world.register_event_handler(robot_event_handler)
    broadcaster.register_listener(robot)
    broadcaster.register_listener(world)
    world.run()

if (__name__ == '__main__'):
    main()
