import ode
import pygame
import os
import OpenGL.GL as ogl
import OpenGL.GLU as oglu
import OpenGL.GLUT as oglut
import orwell.proxy_simulator.maths as maths
import orwell.proxy_simulator.communications as communications


class World(communications.BaseEventHandler):
    cameraDistance = 10.0
    clip = 100.0
    fps = 50.0

    def __init__(self, event_dispatcher, resolution=(1024, 768),
                 draw_helpers=False):
        self._resolution = resolution
        self._event_dispatcher = event_dispatcher
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
        self._event_dispatcher.register_event_handler(self)
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
        oglut.glutInit()

        ogl.glViewport(0, 0, self._resolution[0], self._resolution[1])
        ogl.glClearColor(0.8, 0.8, 0.9, 0)

        ogl.glEnable(ogl.GL_DEPTH_TEST)
        ogl.glEnable(ogl.GL_LIGHTING)
        ogl.glEnable(ogl.GL_NORMALIZE)
        ogl.glShadeModel(ogl.GL_FLAT)
        self._init_textures()

    def _init_textures(self):
        #global texture
        image = pygame.image.load(os.path.join("data", "checker.gif"))

        ix = image.get_width()
        iy = image.get_height()
        image = pygame.image.tostring(image, "RGBX")

        # Create Texture
        texture_id = ogl.glGenTextures(1)
        ogl.glBindTexture(ogl.GL_TEXTURE_2D, texture_id)

        ogl.glTexImage2D(ogl.GL_TEXTURE_2D, 0, 3, ix, iy, 0,
                         ogl.GL_RGBA, ogl.GL_UNSIGNED_BYTE, image)
        ogl.glTexParameteri(ogl.GL_TEXTURE_2D,
                            ogl.GL_TEXTURE_MIN_FILTER,
                            ogl.GL_LINEAR)
        ogl.glTexParameteri(ogl.GL_TEXTURE_2D,
                            ogl.GL_TEXTURE_MAG_FILTER,
                            ogl.GL_LINEAR)

    def _init_ode(self):
        self._world = ode.World()
        self._world.setGravity((0, -9.81, 0))
        self._space = ode.Space()
        self._floor = ode.GeomPlane(self._space,
                                    maths.normalise((0.1, 1, 0)), -2)
        #self._floor = ode.GeomPlane(self._space,
                                    #maths.normalise((0, 1, 0.1)), -2)
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

        ogl.glPushMatrix()
        ogl.glMultMatrixd(self._extract_matrix(geom))

        ogl.glMaterialfv(ogl.GL_FRONT, ogl.GL_SPECULAR, (0.0, 0.0, 0.0))

        if (isinstance(geom, ode.GeomBox)):
            sx, sy, sz = geom.getLengths()
            ogl.glScale(sx, sy, sz)
            oglut.glutSolidCube(1)
        elif (isinstance(geom, ode.GeomSphere)):
            r = geom.getRadius()
            oglut.glutSolidSphere(r, 20, 20)
        elif (isinstance(geom, ode.GeomCylinder)):
            r, h = geom.getParams()
            oglut.glutSolidCylinder(r, h, 20, 20)

        ogl.glPopMatrix()

    def _render_helper(self, helper):
        """
        Draw a sphere.
        `helper`: (coordinates, color). The color is a vector of three
        values in the range [0..1].
        """
        coords, color = helper
        ogl.glPushMatrix()
        ogl.glMultMatrixd((1, 0, 0, 0,
                           0, 1, 0, 0,
                           0, 0, 1, 0,
                           coords[0], coords[1], coords[2], 1))

        ogl.glMaterialfv(ogl.GL_FRONT, ogl.GL_SPECULAR,
                         (color[0], color[1], color[2]))
        oglut.glutSolidSphere(0.2, 20, 20)
        ogl.glPopMatrix()

    def _render_ground(self):
        """
        Renders the ground plane.
        """

        # Draw a quad at the position of the vehicle that extends to the
        # clipping planes.
        ogl.glEnable(ogl.GL_TEXTURE_2D)

        far = 110.0
        normal, d = self._floor.getParams()
        repetitions = 11
        rotation_matrix = maths.make_rotation_matrix(normal)
        if ((self._use_camera_giver) and (self._camera_giver is not None)):
            # move the texture allong with the plane, since it is always drawn
            # at the same relative position to the camera's projection on
            # the plane
            inverse_rotation_matrix = maths.transpose(rotation_matrix)
            camera_pos, _, _ = self._camera_giver.camera
            projected_camera = maths.project_on_plane(normal, d, camera_pos)
            offset_camera = maths.mat_mul(inverse_rotation_matrix,
                                          projected_camera)
            offset_camera = maths.add(offset_camera, maths.vec_mul(normal, d))
            offset_texture = maths.vec_mul(offset_camera,
                                           repetitions / (2 * far))
            otx = offset_texture[0]
            oty = offset_texture[2]
        else:
            offset_camera = (0, 0, 0)
            otx, oty = (0, 0)

        ogl.glPushMatrix()
        #ogl.glTranslate(x, 0.0, z)

        ogl.glMaterialfv(ogl.GL_FRONT, ogl.GL_SPECULAR, (0.0, 1.0, 0.0))

        ogl.glBegin(ogl.GL_QUADS)
        ogl.glColor3f(0.0, 1.0, 0.0)
        ogl.glNormal3f(*normal)
        ogl.glTexCoord2f(0.0 + otx, 0.0 + oty)
        ogl.glVertex3f(
            *maths.add(maths.mat_mul(rotation_matrix,
                                     maths.add((-far, 0, -far),
                                               offset_camera)),
                       maths.vec_mul(normal, d)))
        ogl.glNormal3f(*normal)
        ogl.glTexCoord2f(repetitions + otx, 0.0 + oty)
        ogl.glVertex3f(
            *maths.add(maths.mat_mul(rotation_matrix,
                                     maths.add((far, 0, -far),
                                               offset_camera)),
                       maths.vec_mul(normal, d)))
        ogl.glNormal3f(*normal)
        ogl.glTexCoord2f(repetitions + otx, repetitions + oty)
        ogl.glVertex3f(
            *maths.add(maths.mat_mul(rotation_matrix,
                                     maths.add((far, 0, far),
                                               offset_camera)),
                       maths.vec_mul(normal, d)))
        ogl.glNormal3f(*normal)
        ogl.glTexCoord2f(0.0 + otx, repetitions + oty)
        ogl.glVertex3f(
            *maths.add(maths.mat_mul(rotation_matrix,
                                     maths.add((-far, 0, far),
                                               offset_camera)),
                       maths.vec_mul(normal, d)))
        ogl.glEnd()

        ogl.glPopMatrix()
        ogl.glDisable(ogl.GL_TEXTURE_2D)

    def _set_camera(self):
        """
        Position the camera to C{self.cameraDistance} units behind the
        vehicle's current position and rotated depending on the mouse position.
        """

        aspect = float(self._resolution[0]) / float(self._resolution[1])

        ogl.glMatrixMode(ogl.GL_PROJECTION)
        ogl.glLoadIdentity()
        ogl.glFrustum(-aspect, aspect, -1.0, 1.0, 1.5, self.clip)

        ogl.glLightfv(ogl.GL_LIGHT0, ogl.GL_POSITION, (-5.0, 10.0, 0, 0))
        ogl.glLightfv(ogl.GL_LIGHT0, ogl.GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
        ogl.glLightfv(ogl.GL_LIGHT0, ogl.GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))
        ogl.glEnable(ogl.GL_LIGHT0)

        ogl.glMatrixMode(ogl.GL_MODELVIEW)
        ogl.glLoadIdentity()

        if ((self._camera_giver is None) or (not self._use_camera_giver)):
            x, y = pygame.mouse.get_pos()
            self._xRot = (y - self._resolution[1] / 2) * self._xCoeff
            self._yRot = (x - self._resolution[0] / 2) * self._yCoeff
            if (self._xRot < 0):
                self._xRot = 0

            # Set the camera angle to view the vehicle
            ogl.glTranslate(0.0, 0.0, -self.cameraDistance)
            ogl.glRotate(self._xRot, 1, 0, 0)
            ogl.glRotate(self._yRot, 0, 1, 0)
        else:
            pos, at, up = self._camera_giver.camera
            self._helpers.append((pos, (1, 0, 0)))
            self._helpers.append((at, (0, 1, 0)))
            self._helpers.append((maths.add(pos, up), (0, 0, 1)))
            oglu.gluLookAt(pos[0], pos[1], pos[2],
                           at[0], at[1], at[2],
                           up[0], up[1], up[2])

        ## Set the camera so that the vehicle is drawn in the correct place.
        #x, y, z = self.chassis.getPosition()
        #ogl.glTranslate(-x, -y, -z)

    def render(self):
        """
        Render the current simulation state.
        """

        ogl.glClear(ogl.GL_COLOR_BUFFER_BIT | ogl.GL_DEPTH_BUFFER_BIT)
        self._render_ground()

        self._set_camera()
        #print "len(self._geoms) =", len(self._geoms)
        for geom in self._geoms:
            self._render_geom(geom)

        if (self._draw_helpers):
            for helper in self._helpers:
                self._render_helper(helper)
        del self._helpers[:]

        ogl.glFlush()
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
        elif (key == pygame.K_h):
            self._draw_helpers = not self._draw_helpers

    def _key_up(self, key):
        if (key in (pygame.K_q, pygame.K_a)):
            self._velocity_left = 0.0
        elif (key in (pygame.K_e, pygame.K_d)):
            self._velocity_right = 0.0

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
                    axis = maths.cross_product(normal, wheel_axis_world)
                    axis = maths.normalise(axis)
                    if (axis is not None):
                        wheel_axis_floor = maths.cross_product(axis, normal)
                        self._helpers.append((maths.add(normal, pos),
                                              (0, 1, 0)))
                        self._helpers.append((maths.add(wheel_axis_world, pos),
                                              (0, 0, 1)))
                        self._helpers.append((maths.add(axis, pos),
                                              (1, 0, 0)))
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

    def step(self):
        """
        Perform a single step. This means processing the events, updating the
        robots, stepping the physical simulation and rendering the scene.
        This method is called in a loop by run, but you may call it from
        outside, if you want to provide your own loop.
        """
        self._event_dispatcher.step()

        for robot in self._robots:
            #robot.velocity_left = self._velocity_left
            #robot.velocity_right = self._velocity_right
            robot.update()

        self._space.collide((), self._nearcb)
        self._world.step(1 / self.fps)
        self._cjoints.empty()
        self.render()

    def run(self):
        """
        Start the demo. This method will block until the demo exits.
        """

        clock = pygame.time.Clock()
        self._running = True

        while self._running:
            self.step()

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
