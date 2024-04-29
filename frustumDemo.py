
#from panda3d.core import loadPrcFileData
#loadPrcFileData("", "want-directtools #t")
#loadPrcFileData("", "want-tk #t")
import pybullet as p
import numpy as np
import time
import math
import pybullet_data
from frustum import *
from math import pi, sin, cos
from direct.showbase.ShowBase import ShowBase
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
from wind import *



class MyApp(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)
        self.worldNP = render.attachNewNode('World')

        # World
        self.debugNP = self.worldNP.attachNewNode(BulletDebugNode('Debug'))
        self.debugNP.show()
        self.debugNP.node().showWireframe(True)
        self.debugNP.node().showConstraints(True)
        self.debugNP.node().showBoundingBoxes(False)
        self.debugNP.node().showNormals(True)

        #self.debugNP.showTightBounds()
        #self.debugNP.showBounds()

        self.world = BulletWorld()
        self.world.setGravity(Vec3(0, 0, -9.81))
        self.world.setDebugNode(self.debugNP.node())
        
        #wind simulation
        self.wind_simulator = Wind(direction = Vec3(1,0,0), magnitude=1, scale=0.1)

        # Ground (static)
        shape = BulletPlaneShape(Vec3(0, 0, 1), 1)

        self.groundNP = self.worldNP.attachNewNode(BulletRigidBodyNode('Ground'))
        self.groundNP.node().addShape(shape)
        self.groundNP.setPos(0, 0, -2)
        self.groundNP.setCollideMask(BitMask32.allOn())
        

        self.world.attachRigidBody(self.groundNP.node())

        #Frustum (Dynamic)
        # First Frustum Geometry Created 
        frustum = Frustum(np.array([0.0, 0.0, 0.0]), 5.0, np.array([0.0, 0.0, 5.0]), 3.0, None, None, None, None)
        geom = frustum.generateMesh(20)
        mesh = BulletTriangleMesh()
        mesh.addGeom(geom)
        shape = BulletTriangleMeshShape(mesh, dynamic=True)

        #Physical Node Attached To Physics Engine 
        self.frusNP = self.worldNP.attachNewNode(BulletRigidBodyNode('Frustum'))
        self.frusNP.node().setMass(0.0)
        self.frusNP.node().addShape(shape)
        #self.boxNP.setScale(2, 1, 0.5)
        self.frusNP.setCollideMask(BitMask32.allOn())
        #self.boxNP.node().setDeactivationEnabled(False)

        self.world.attachRigidBody(self.frusNP.node())

        # Visual Node Created With Texture And Attached to Renderer 
        myTexture = loader.loadTexture("Pinewood_Bark_DIFF.png")
        node = GeomNode('FRUSTUMNODE')
        node.addGeom(geom)
        self.nodePath = self.render.attachNewNode(node)
        self.nodePath.setTexture(myTexture)
        self.nodePath.reparentTo(self.frusNP)
        self.cam.setPos(0, -60, 55)
        self.cam.lookAt(self.nodePath)
        self.world.setDebugNode(self.debugNP.node())

        # Second Physical Node Created And Attached To Physics Engine 
        frustum1 = Frustum(np.array([0.0, 0.0, 5.0]), 3.0, np.array([0.0, 0.0, 8.0]), 2.5, None, None, None, None)
        geom1 = frustum1.generateMesh(20)
        mesh = BulletTriangleMesh()
        mesh.addGeom(geom)
        shape = BulletTriangleMeshShape(mesh, dynamic=True)
        self.frus1NP = self.worldNP.attachNewNode(BulletRigidBodyNode('PhysFrustum1'))
        self.frus1NP.node().setMass(1000.0)
        self.frus1NP.node().addShape(shape)
        self.frus1NP.setCollideMask(BitMask32.allOn())

        self.world.attachRigidBody(self.frus1NP.node())

        # Second Visual Node Created And Attached To First Visual Node 
        node1 = GeomNode('FRUSTUMNODE')
        node1.addGeom(geom1)
        self.nodePath1 = self.nodePath.attachNewNode(node1)
        self.nodePath1.setTexture(myTexture)
        self.nodePath1.reparentTo(self.frus1NP)

        #Setup ConeTwistConstraint Between Two Frustums 
        frame1 = TransformState.makeHpr(Vec3(0, 0, 0)) # Positions Are Already in place, so is orientation so we can set Haw:0, Pitch:0  Roll: 0
        frame2 = TransformState.makeHpr(Vec3(0, 0, 0))
        #Set Allowable Cone Twistt Angles 
        swing1 = 60 # degrees 
        swing2 = 36 # degrees
        twist = 120 # degrees

        # Add Constraint With Limits to Physics Engine
        cs = BulletConeTwistConstraint(self.frusNP.node(), self.frus1NP.node(), frame1, frame2)
        cs.setDebugDrawSize(2.0)
        cs.setLimit(swing1, swing2, twist)
        cs.setDamping(0.3)
        self.world.attachConstraint(cs)

        #Camera Set To Look At Node1
        self.cam.setPos(0, -60, 55)
        self.cam.lookAt(self.nodePath1)
        self.world.setDebugNode(self.debugNP.node())


        myTexture = loader.loadTexture("Pinewood_Bark_DIFF.png")
        node = GeomNode('FRUSTUMNODE')
        node.addGeom(geom)
        self.nodePath = self.render.attachNewNode(node)
        self.nodePath.setTexture(myTexture)
        self.nodePath.reparentTo(self.frusNP)
        self.cam.setPos(0, -60, 55)
        self.cam.lookAt(self.nodePath)
        self.world.setDebugNode(self.debugNP.node())
        
        self.taskMgr.add(self.update, 'updatePhysics')
    
    def update(self, task):
      dt = globalClock.getDt()
      wind_force = self.wind_simulator.get_wind_force()
      for node in [self.frusNP.node(), self.frus1NP.node()]:
        drag_force = self.wind_simulator.get_drag_force(node)
        total_force = wind_force+drag_force
        node.applyCentralForce(total_force)
      self.world.doPhysics(dt, 1, 1)
      return Task.cont

    


app = MyApp()
app.run()
