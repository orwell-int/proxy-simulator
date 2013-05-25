import ode
import orwell.proxy_simulator.tanks as tanks


class Tank3(tanks.BaseTank):

    def __init__(self, robot_descriptor):
        super(Tank3, self).__init__(robot_descriptor)

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


class TankWithSpheres(tanks.BaseTank):
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
