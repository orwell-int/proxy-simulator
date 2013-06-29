import ode
import math
import orwell.proxy_simulator.maths as maths
import orwell.proxy_simulator.version1_pb2 as pb_messages


class BaseTank(object):

    def __init__(self, robot_descriptor):
        self._velocity_left = 0
        self._velocity_right = 0
        self._robot_descriptor = robot_descriptor
        self._left_wheel_joints = []
        self._right_wheel_joints = []
        self._chassis_body = None

    def create_objects(self, world):
        pass

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

    def handle_message(self, recipient, payload):
        if (self._robot_descriptor.recipient == recipient):
            wrapper_msg = pb_messages.base_message()
            try:
                wrapper_msg.ParseFromString(payload)
            except Exception as e:
                print e
            else:
                left, right = \
                        self._robot_descriptor.get_movement(wrapper_msg)
                self.velocity_left = 10 * left
                self.velocity_right = 10 * right

    @property
    def camera(self):
        up_local = (0, 3, 0)
        up_world = self._chassis_body.vectorToWorld(up_local)
        camera_position = self._chassis_body.getPosition()
        camera_position = maths.add(camera_position, up_world)
        camera_at = self._chassis_body.vectorToWorld((0, 0, 1))
        camera_at = maths.add(camera_position, camera_at)
        up_local = (0, 1, 0)
        up_world = self._chassis_body.vectorToWorld(up_local)
        return (camera_position, camera_at, up_world)


class Tank(BaseTank):
    def __init__(self, robot_descriptor):
        super(Tank, self).__init__(robot_descriptor)

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
    """
    Handles the messaging aspects of the tanks. There will be code to
    factorise when more robots are added, and probably some reorganisation.
    """
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
        return wrapper_msg.SerializeToString()

    def get_movement(self, wrapper_msg):
        assert("MOVE_TANK" == wrapper_msg.message_type)
        sub_msg = pb_messages.move_tank_message()
        sub_msg.ParseFromString(wrapper_msg.serialized_message)
        assert(-1 <= sub_msg.left <= 1)
        assert(-1 <= sub_msg.right <= 1)
        return (sub_msg.left, sub_msg.right)
