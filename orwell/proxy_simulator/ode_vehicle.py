import ode
import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import orwell.proxy_simulator.version1_pb2 as pb_messages


class Tank(object):

    def __init__(self, robot_descriptor):
        self._velocity_left = 0
        self._velocity_right = 0
        self._robot_descriptor = robot_descriptor
        self._left_wheel_joints = []
        self._right_wheel_joints = []
        self._chassis_body = None

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
        self._chassis_body = body

        density = 1
        print 'left wheel'
        # left wheel
        radius = 1
        height = 0.2
        px, py, pz = (lx / 2, 0, -(lz / 2))
        left_wheel_body = ode.Body(world.world)
        wheel_mass = ode.Mass()
        #wheel_mass.setSphere(density, radius)
        wheel_mass.setCylinder(density, 1, radius, height)
        left_wheel_body.setMass(wheel_mass)
        #left_wheel_geom = ode.GeomSphere(world.space, radius=radius)
        left_wheel_geom = ode.GeomCylinder(world.space, radius=radius,
                                           length=height)
        left_wheel_geom.setBody(left_wheel_body)
        #left_wheel_body.setPosition((px, py, pz))
        left_wheel_body.setRotation((0, 0, 1,
                                     0, 1, 0,
                                     -1, 0, 0))
        left_wheel_body.setPosition((px - height / 2, py, pz))
        world.add_body(left_wheel_body)
        world.add_geom(left_wheel_geom)

        print 'right wheel'
        # right wheel
        #radius = 1
        px = -lx / 2
        right_wheel_body = ode.Body(world.world)
        wheel_mass = ode.Mass()
        #wheel_mass.setSphere(density, radius)
        wheel_mass.setCylinder(density, 1, radius, height)
        right_wheel_body.setMass(wheel_mass)
        #right_wheel_geom = ode.GeomSphere(world.space, radius=radius)
        right_wheel_geom = ode.GeomCylinder(world.space, radius=radius,
                                            length=height)
        right_wheel_geom.setBody(right_wheel_body)
        #right_wheel_body.setPosition((px, py, pz))
        right_wheel_body.setRotation((0, 0, 1,
                                      0, 1, 0,
                                      -1, 0, 0))
        right_wheel_body.setPosition((px - height / 2, py, pz))
        world.add_body(right_wheel_body)
        world.add_geom(right_wheel_geom)

        print 'front wheel'
        # front wheel
        #radius = 1
        px, py, pz = (0, 0, lz / 2)
        front_wheel_body = ode.Body(world.world)
        wheel_mass = ode.Mass()
        wheel_mass.setSphere(density, radius)
        front_wheel_body.setMass(wheel_mass)
        front_wheel_geom = ode.GeomSphere(world.space, radius=radius)
        front_wheel_geom.setBody(front_wheel_body)
        front_wheel_body.setPosition((px, py, pz))
        world.add_body(front_wheel_body)
        world.add_geom(front_wheel_geom)

        #left_wheel_joint = ode.Hinge2Joint(world.world)
        left_wheel_joint = ode.HingeJoint(world.world)
        left_wheel_joint.attach(body, left_wheel_body)
        left_wheel_joint.setAnchor(left_wheel_body.getPosition())
        left_wheel_joint.setAxis((-1, 0, 0))
        #left_wheel_joint.setAxis1((0, 1, 0))
        #left_wheel_joint.setAxis2((1, 0, 0))
        left_wheel_joint.setParam(ode.ParamFMax, 500000)
        #left_wheel_joint.setParam(ode.ParamLoStop, 0)
        #left_wheel_joint.setParam(ode.ParamHiStop, 0)
        #left_wheel_joint.setParam(ode.ParamFMax2, 0.1)
        #left_wheel_joint.setParam(ode.ParamSuspensionERP, 0.2)
        #left_wheel_joint.setParam(ode.ParamSuspensionCFM, 0.1)
        self._left_wheel_joints.append(left_wheel_joint)

        #right_wheel_joint = ode.Hinge2Joint(world.world)
        right_wheel_joint = ode.HingeJoint(world.world)
        right_wheel_joint.attach(body, right_wheel_body)
        right_wheel_joint.setAnchor(right_wheel_body.getPosition())
        right_wheel_joint.setAxis((-1, 0, 0))
        #right_wheel_joint.setAxis1((0, 1, 0))
        #right_wheel_joint.setAxis2((1, 0, 0))
        right_wheel_joint.setParam(ode.ParamFMax, 500000)
        #right_wheel_joint.setParam(ode.ParamLoStop, 0)
        #right_wheel_joint.setParam(ode.ParamHiStop, 0)
        #right_wheel_joint.setParam(ode.ParamFMax2, 0.1)
        #right_wheel_joint.setParam(ode.ParamSuspensionERP, 0.2)
        #right_wheel_joint.setParam(ode.ParamSuspensionCFM, 0.1)
        self._right_wheel_joints.append(right_wheel_joint)

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
        for left_wheel_joint in self._left_wheel_joints:
            left_wheel_joint.setParam(ode.ParamVel, self._velocity_left)
        for right_wheel_joint in self._right_wheel_joints:
            right_wheel_joint.setParam(ode.ParamVel, self._velocity_right)

    def handle_message(self, wrapper_msg):
        if (self._robot_descriptor.recipient == wrapper_msg.recipient):
            left, right = \
                    self._robot_descriptor.get_movement(wrapper_msg)
            self.velocity_left = 10 * left
            self.velocity_right = 10 * right

    @property
    def camera(self):
        up_local = (0, 3, 0)
        up_world = self._chassis_body.vectorToWorld(up_local)
        camera_position = self._chassis_body.getPosition()
        camera_position = add(camera_position, up_world)
        camera_at = self._chassis_body.vectorToWorld((0, 0, 1))
        camera_at = add(camera_position, camera_at)
        up_local = (0, 1, 0)
        up_world = self._chassis_body.vectorToWorld(up_local)
        return (camera_position, camera_at, up_world)


class TankWithSpheres(Tank):
    def __init__(self, robot_descriptor):
        super(TankWithSpheres, self).__init__(robot_descriptor)

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
        self._chassis_body = body

        density = 1
        print 'left wheel'
        # left wheel
        radius = 1
        height = 0.2
        px, py, pz = (lx / 2, 0, -(lz / 2))
        left_wheel_body = ode.Body(world.world)
        wheel_mass = ode.Mass()
        wheel_mass.setSphere(density, radius)
        left_wheel_body.setMass(wheel_mass)
        left_wheel_geom = ode.GeomSphere(world.space, radius=radius)
        left_wheel_geom.setBody(left_wheel_body)
        left_wheel_body.setPosition((px, py, pz))
        world.add_body(left_wheel_body)
        world.add_geom(left_wheel_geom)

        print 'right wheel'
        # right wheel
        #radius = 1
        px = -(lx / 2)
        right_wheel_body = ode.Body(world.world)
        wheel_mass = ode.Mass()
        wheel_mass.setSphere(density, radius)
        right_wheel_body.setMass(wheel_mass)
        right_wheel_geom = ode.GeomSphere(world.space, radius=radius)
        right_wheel_geom.setBody(right_wheel_body)
        right_wheel_body.setPosition((px, py, pz))
        world.add_body(right_wheel_body)
        world.add_geom(right_wheel_geom)

        print 'front wheel'
        # front wheel
        #radius = 1
        px, py, pz = (0, 0, lz / 2)
        front_wheel_body = ode.Body(world.world)
        wheel_mass = ode.Mass()
        wheel_mass.setSphere(density, radius)
        front_wheel_body.setMass(wheel_mass)
        front_wheel_geom = ode.GeomSphere(world.space, radius=radius)
        front_wheel_geom.setBody(front_wheel_body)
        front_wheel_body.setPosition((px, py, pz))
        world.add_body(front_wheel_body)
        world.add_geom(front_wheel_geom)

        left_wheel_joint = ode.HingeJoint(world.world)
        left_wheel_joint.attach(body, left_wheel_body)
        left_wheel_joint.setAnchor(left_wheel_body.getPosition())
        left_wheel_joint.setAxis((-1, 0, 0))
        left_wheel_joint.setParam(ode.ParamFMax, 500000)
        self._left_wheel_joints.append(left_wheel_joint)

        right_wheel_joint = ode.HingeJoint(world.world)
        right_wheel_joint.attach(body, right_wheel_body)
        right_wheel_joint.setAnchor(right_wheel_body.getPosition())
        right_wheel_joint.setAxis((-1, 0, 0))
        right_wheel_joint.setParam(ode.ParamFMax, 500000)
        self._right_wheel_joints.append(right_wheel_joint)

        front_wheel_joint = ode.BallJoint(world.world)
        front_wheel_joint.attach(body, front_wheel_body)
        front_wheel_joint.setAnchor(front_wheel_body.getPosition())
        front_wheel_joint.setParam(ode.ParamFMax, 5000)


class TankWithCheapTracks(Tank):
    def __init__(self, robot_descriptor):
        super(TankWithCheapTracks, self).__init__(robot_descriptor)

    def create_objects(self, world):
        print 'chassis'
        fmax = 5000 * 1.6
        scale = 0.04
        # chassis
        density = 50.5
        lx, ly, lz = (145 * scale, 10 * scale, 177 * scale)
        # Create body
        body = ode.Body(world.world)
        mass = ode.Mass()
        mass.setBox(density, lx, ly, lz)
        body.setMass(mass)
        chassis_mass = lx * ly * lz * density
        print "chassis mass =", chassis_mass

        # Set parameters for drawing the body
        body.shape = "box"
        body.boxsize = (lx, ly, lz)

        # Create a box geom for collision detection
        geom = ode.GeomBox(world.space, lengths=body.boxsize)
        geom.setBody(body)
        #body.setPosition((0, 3, 0))
        world.add_body(body)
        world.add_geom(geom)
        self._chassis_body = body

        density = 4
        # left wheel
        radius = 25 * scale
        height = radius * 0.8
        wheel_mass_ = math.pi * radius ** 2 * height * density
        print "wheel mass =", wheel_mass_
        px = lx / 2
        py = 0
        print 'left wheels'
        for pz in (-(lz / 2), 0, lz / 2):
            # cylinders
            left_wheel_body = ode.Body(world.world)
            wheel_mass = ode.Mass()
            wheel_mass.setCylinder(density, 1, radius, height)
            left_wheel_body.setMass(wheel_mass)
            left_wheel_geom = ode.GeomCylinder(world.space, radius=radius,
                                               length=height)
            left_wheel_geom.setBody(left_wheel_body)
            left_wheel_body.setRotation((0, 0, 1,
                                         0, 1, 0,
                                         -1, 0, 0))
            left_wheel_body.setPosition((px - height / 2, py, pz))

            # spheres
            #left_wheel_body = ode.Body(world.world)
            #wheel_mass = ode.Mass()
            #wheel_mass.setSphere(density, radius)
            #left_wheel_body.setMass(wheel_mass)
            #left_wheel_geom = ode.GeomSphere(world.space, radius=radius)
            #left_wheel_geom.setBody(left_wheel_body)
            #left_wheel_body.setPosition((px, py, pz))

            world.add_body(left_wheel_body)
            world.add_geom(left_wheel_geom)

            left_wheel_joint = ode.HingeJoint(world.world)
            left_wheel_joint.attach(body, left_wheel_body)
            left_wheel_joint.setAnchor(left_wheel_body.getPosition())
            left_wheel_joint.setAxis((-1, 0, 0))
            left_wheel_joint.setParam(ode.ParamFMax, fmax)
            self._left_wheel_joints.append(left_wheel_joint)

        print 'right wheels'
        # right wheel
        px = -(lx / 2)
        for pz in (-(lz / 2), 0, lz / 2):
            # cylinders
            right_wheel_body = ode.Body(world.world)
            wheel_mass = ode.Mass()
            wheel_mass.setCylinder(density, 1, radius, height)
            right_wheel_body.setMass(wheel_mass)
            right_wheel_geom = ode.GeomCylinder(world.space, radius=radius,
                                               length=height)
            right_wheel_geom.setBody(right_wheel_body)
            right_wheel_body.setRotation((0, 0, 1,
                                          0, 1, 0,
                                          -1, 0, 0))
            right_wheel_body.setPosition((px - height / 2, py, pz))

            # spheres
            #right_wheel_body = ode.Body(world.world)
            #wheel_mass = ode.Mass()
            #wheel_mass.setSphere(density, radius)
            #right_wheel_body.setMass(wheel_mass)
            #right_wheel_geom = ode.GeomSphere(world.space, radius=radius)
            #right_wheel_geom.setBody(right_wheel_body)
            #right_wheel_body.setPosition((px, py, pz))

            world.add_body(right_wheel_body)
            world.add_geom(right_wheel_geom)

            right_wheel_joint = ode.HingeJoint(world.world)
            right_wheel_joint.attach(body, right_wheel_body)
            right_wheel_joint.setAnchor(right_wheel_body.getPosition())
            right_wheel_joint.setAxis((-1, 0, 0))
            right_wheel_joint.setParam(ode.ParamFMax, fmax)
            self._right_wheel_joints.append(right_wheel_joint)

        print "total mass =", float(chassis_mass + 6 * wheel_mass_)


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


def cross_product(v1, v2):
    return (
            v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]
            )


def normalise(vector):
    if (all(((0 == coord) for coord in vector))):
        return None
    length_p2 = vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2
    length = length_p2 ** 0.5
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def project_on_plane(normal, distance, point):
    """
    Project a point on a plance defined by a normal and a distance to the
    origin following a line parallel to the normal.
    equation of the plane:
        normal[0] * x + normal[1] * y + normal[2] * z + distance = 0
    `normal`: normal of the plance.
    `distance`: distance to the origin.
    `point`: the point to project (projection parallel to the normal).
    """
    up = (normal[0] * point[0] + normal[1] * point[1] + normal[2] * point[2])
    down = normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2
    t = - up / down
    projected = (
        point[0] + normal[0] * t,
        point[1] + normal[1] * t,
        point[2] + normal[2] * t
    )
    return projected


def add(v1, v2):
    return (v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2])


def vec_mul(vector, factor):
    return (vector[0] * factor, vector[1] * factor, vector[2] * factor)


class World(BaseEventHandler):
    cameraDistance = 10.0
    clip = 100.0
    fps = 50.0

    def __init__(self, broadcaster, resolution=(1024, 768),
                 draw_helpers=False):
        self._resolution = resolution
        self._broadcaster = broadcaster
        self._draw_helpers = draw_helpers
        self._camera_giver = None
        self._use_camera_giver = False
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
        self._helpers = []

    @property
    def camera_giver(self):
        return self._camera_giver

    @camera_giver.setter
    def camera_giver(self, value):
        self._camera_giver = value

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
        self._init_textures()

    def _init_textures(self):
        #global texture
        image = pygame.image.load(os.path.join("data", "checker.gif"))

        ix = image.get_width()
        iy = image.get_height()
        image = pygame.image.tostring(image, "RGBX")

        # Create Texture
        texture_id = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, texture_id)

        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, image)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

    def _init_ode(self):
        self._world = ode.World()
        self._world.setGravity((0, -9.81, 0))
        self._space = ode.Space()
        self._floor = ode.GeomPlane(self._space, (0.1, 1, 0), -2)
        #self._floor = ode.GeomPlane(self._space, (0, 1, 0.1), -2)
        #self._floor = ode.GeomPlane(self._space, (0, 1, 0), -1)

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

        #allowed = [ode.GeomSphere, ode.GeomCylinder]
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

    def _render_helper(self, helper):
        """
        """
        coords, color = helper
        glPushMatrix()
        glMultMatrixd((1, 0, 0, 0,
                       0, 1, 0, 0,
                       0, 0, 1, 0,
                       coords[0], coords[1], coords[2], 1))

        glMaterialfv(GL_FRONT, GL_SPECULAR, (color[0], color[1], color[2]))
        glutSolidSphere(0.2, 20, 20)
        glPopMatrix()

    def _render_ground(self):
        """
        Renders the ground plane.
        """

        # Draw a quad at the position of the vehicle that extends to the
        # clipping planes.
        glEnable(GL_TEXTURE_2D)

        normal, d = self._floor.getParams()
        #x, y, z = self.chassis.getPosition()
        repetitions = 10

        glPushMatrix()
        #glTranslate(x, 0.0, z)

        glMaterialfv(GL_FRONT, GL_SPECULAR, (0.0, 1.0, 0.0))

        glBegin(GL_QUADS)
        glColor3f(0.0, 1.0, 0.0)
        glNormal3f(*normal)
        glTexCoord2f(0.0, 0.0)
        glVertex3f(*add(
            project_on_plane(normal, d, (-self.clip, 0, -self.clip)),
            vec_mul(normal, d)))
        #glVertex3f(-self.clip, d, -self.clip)
        glNormal3f(*normal)
        glTexCoord2f(repetitions, 0.0)
        glVertex3f(*add(
            project_on_plane(normal, d, (self.clip, 0, -self.clip)),
            vec_mul(normal, d)))
        #glVertex3f(self.clip, d, -self.clip)
        glNormal3f(*normal)
        glTexCoord2f(repetitions, repetitions)
        glVertex3f(*add(
            project_on_plane(normal, d, (self.clip, 0, self.clip)),
            vec_mul(normal, d)))
        #glVertex3f(self.clip, d, self.clip)
        glNormal3f(*normal)
        glTexCoord2f(0.0, repetitions)
        glVertex3f(*add(
            project_on_plane(normal, d, (-self.clip, 0, self.clip)),
            vec_mul(normal, d)))
        #glVertex3f(-self.clip, d, self.clip)
        glEnd()

        glPopMatrix()
        glDisable(GL_TEXTURE_2D)

    def _set_camera(self):
        """
        Position the camera to C{self.cameraDistance} units behind the
        vehicle's current position and rotated depending on the mouse position.
        """

        aspect = float(self._resolution[0]) / float(self._resolution[1])

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-aspect, aspect, -1.0, 1.0, 1.5, self.clip)

        glLightfv(GL_LIGHT0, GL_POSITION, (-5.0, 10.0, 0, 0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))
        glEnable(GL_LIGHT0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        if ((self._camera_giver is None) or (not self._use_camera_giver)):
            x, y = pygame.mouse.get_pos()
            self._xRot = (y - self._resolution[1] / 2) * self._xCoeff
            self._yRot = (x - self._resolution[0] / 2) * self._yCoeff
            if (self._xRot < 0):
                self._xRot = 0

            # Set the camera angle to view the vehicle
            glTranslate(0.0, 0.0, -self.cameraDistance)
            glRotate(self._xRot, 1, 0, 0)
            glRotate(self._yRot, 0, 1, 0)
        else:
            pos, at, up = self._camera_giver.camera
            self._helpers.append((pos, (1, 0, 0)))
            self._helpers.append((at, (0, 1, 0)))
            self._helpers.append((add(pos, up), (0, 0, 1)))
            gluLookAt(pos[0], pos[1], pos[2], at[0], at[1], at[2],
                      up[0], up[1], up[2])

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

        if (self._draw_helpers):
            for helper in self._helpers:
                self._render_helper(helper)
        del self._helpers[:]

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
        elif(key == pygame.K_SPACE):
            self._use_camera_giver = not self._use_camera_giver

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

    def _make_simple_contact(self, contact, body1, body2):
        contact.setBounce(0.2)
        contact.setMu(10000)
        joint = ode.ContactJoint(self.world, self._cjoints, contact)
        joint.attach(body1, body2)

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

        floor_found = False
        other_geom = None
        for geom in (geom1, geom2):
            if (floor_found):
                other_geom = geom
            if (self._floor == geom):
                floor_found = True
            else:
                other_geom = geom
        contacts_generated = False
        if (floor_found):
            allowed = (ode.GeomSphere, ode.GeomCylinder)
            ok = False
            for klass in allowed:
                ok = ok or isinstance(other_geom, klass)
            if (ok):
                # we have found a wheel touching the floor
                body = other_geom.getBody()
                wheel_axis_local = (0, 0, 1)
                wheel_axis_world = body.vectorToWorld(wheel_axis_local)
                #wheel_axis_world = body.getRelPointPos(
                        #body.vectorToWorld(wheel_axis_local))
                #print "wheel_axis_world =", wheel_axis_world
                #self._helpers.append((wheel_axis_world, (1, 0, 0)))
                #wheel_axis_world = (
                        #body.getPosition()[0] - wheel_axis_world[0],
                        #body.getPosition()[1] - wheel_axis_world[1],
                        #body.getPosition()[2] - wheel_axis_world[2]
                        #)
                #print "wheel_axis_world =", wheel_axis_world
                for contact in contacts:
                    pos, normal, _, _, _ = contact.getContactGeomParams()
                    axis = cross_product(normal, wheel_axis_world)
                    axis = normalise(axis)
                    if (axis is not None):
                        wheel_axis_floor = cross_product(axis, normal)
                        #self._helpers.append((add(normal, pos),
                                              #(0, 1, 0)))
                        #self._helpers.append((add(wheel_axis_world, pos),
                                              #(0, 0, 1)))
                        #self._helpers.append((add(axis, pos),
                                              #(1, 0, 0)))
                        contact.setFDir1(wheel_axis_floor)
                        #contact.setFDir1(axis)
                        contact.setMu(6000)
                        contact.setMu2(10000)
                        contact.setBounce(0.2)
                        contact.setMode(ode.ContactFDir1 + ode.ContactMu2)
                        joint = ode.ContactJoint(
                            self.world, self._cjoints, contact)
                        joint.attach(body1, body2)
                    else:
                        self._make_simple_contact(contact, body1, body2)
                contacts_generated = True

        if (not contacts_generated):
            for c in contacts:
                self._make_simple_contact(c, body1, body2)
                #c.setBounce(0.2)
                #c.setMu(10000)
                #j = ode.ContactJoint(self.world, self._cjoints, c)
                #j.attach(body1, body2)

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
    world = World(broadcaster, draw_helpers=True)
    descriptor = TankDescriptor(0)
    robot = TankWithCheapTracks(descriptor)
    robot_event_handler = TankEventHandler(descriptor)
    world.add(robot)
    world.register_event_handler(robot_event_handler)
    world.camera_giver = robot
    broadcaster.register_listener(robot)
    broadcaster.register_listener(world)
    world.run()

if (__name__ == '__main__'):
    main()
