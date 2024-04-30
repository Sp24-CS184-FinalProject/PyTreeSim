import pybullet as p
import numpy as np
import time
import math
import pybullet_data
from frustum import *
from math import pi, sin, cos
from direct.showbase.ShowBase import ShowBase
from direct.gui.DirectGui import DirectButton
from panda3d.core import Vec3, BitMask32
from direct.showbase.DirectObject import DirectObject
from direct.task import Task
from panda3d.core import GeomNode
from panda3d.core import NodePath
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletBoxShape
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletConeTwistConstraint
from panda3d.core import AmbientLight
from panda3d.core import DirectionalLight
from panda3d.core import Vec3
from panda3d.core import Vec4
from panda3d.core import Point3
from panda3d.core import TransformState
from panda3d.core import BitMask32
from tree import *
from wind import *

class MyApp(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)
        # self.setup_gui()
        # self.world_loaded = False
        self.setup_world()

    def setup_gui(self):
        # Create a button to start the simulation
        self.start_button = DirectButton(text="Start Simulation", scale=0.1, command=self.start_simulation)

    def start_simulation(self):
        # Load the world only if it hasn't been loaded yet
        if not self.world_loaded:
            self.setup_world()
            self.world_loaded = True
        # Attach the update task
        self.taskMgr.add(self.update, 'update_world')

    def setup_world(self):
        # World
        self.worldNP = self.render.attachNewNode('World')
        self.debugNP = self.worldNP.attachNewNode(BulletDebugNode('Debug'))
        # self.debugNP.show()
        # self.debugNP.node().showWireframe(False)
        # self.debugNP.node().showConstraints(True)
        # self.debugNP.node().showBoundingBoxes(False)
        # self.debugNP.node().showNormals(True)

        #wind simulation
        self.wind_simulator = Wind(direction = Vec3(1,0,0), magnitude=.5, scale=1.0)

        self.world = BulletWorld()
        self.world.setGravity(Vec3(0, 0, 0))
        self.world.setDebugNode(self.debugNP.node())

        # Ground (static)
        shape = BulletPlaneShape(Vec3(0, 0, 1), 1)

        self.groundNP = self.worldNP.attachNewNode(BulletRigidBodyNode('Ground'))
        self.groundNP.node().addShape(shape)
        self.groundNP.setPos(0, 0, -2)
        self.groundNP.setCollideMask(BitMask32.allOn())

        self.world.attachRigidBody(self.groundNP.node())

        # create a few trees of different types
        myTexture = loader.loadTexture("lowResBark.png")
        height = 10.0
        baseRadius = .75
        
        type = 'original'
        baseOrigin = np.array([0, 0, 0])
        self.original = Tree(type, height, baseOrigin, baseRadius, myTexture, self.worldNP, self.world, self.render)

        # type = 'dense'
        # baseOrigin = np.array([20, 0, 0])
        # dense = Tree(type, height, baseOrigin, baseRadius, myTexture, self.worldNP, self.world, self.render)

        # type = 'sparse'
        # baseOrigin = np.array([40, 0, 0])
        # sparse = Tree(type, height, baseOrigin, baseRadius, myTexture, self.worldNP, self.world, self.render)

        # type = None
        # baseOrigin = np.array([60, 0, 0])
        # default = Tree(type, height, baseOrigin, baseRadius, myTexture, self.worldNP, self.world, self.render)

        #Camera Set To Look At Node1
        self.cam.setPos(0, -50, 0)
        
        self.world.setDebugNode(self.debugNP.node())


        
        #self.cam.setPos(0, -100, 100)
        self.cam.lookAt(self.worldNP)
        self.world.setDebugNode(self.debugNP.node())
        self.taskMgr.add(self.update, 'updatePhysics')
    
    def update(self, task):
      dt = globalClock.getDt()
      wind_force = self.wind_simulator.get_wind_force()
      for node in self.original.getPhysicsNodes():
        drag_force = self.wind_simulator.get_drag_force(node)
        total_force = wind_force+drag_force
        if total_force.is_nan():
           continue
        node.applyCentralForce(total_force)
        node.setLinearVelocity(Vec3(0,0,0)) # Node Is Attached To A Tree And Should Have No Linear Velocity
      self.world.doPhysics(dt, 1, 1)
      return Task.cont

app = MyApp()
app.run()
