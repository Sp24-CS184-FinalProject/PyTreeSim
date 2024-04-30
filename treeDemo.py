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
from tree import *

class MyApp(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)
        self.worldNP = self.render.attachNewNode('World')

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

        # Ground (static)
        shape = BulletPlaneShape(Vec3(0, 0, 1), 1)

        self.groundNP = self.worldNP.attachNewNode(BulletRigidBodyNode('Ground'))
        self.groundNP.node().addShape(shape)
        self.groundNP.setPos(0, 0, -2)
        self.groundNP.setCollideMask(BitMask32.allOn())

        self.world.attachRigidBody(self.groundNP.node())

        # Create A Tree Trunk 
        baseOrigin = np.array([0, 0 , 0])
        baseRadius = 5.0
        height = 100.0
        slimFactor = .98
        frustumMass = 10.0
        myTexture = loader.loadTexture("Pinewood_Bark_DIFF.png")
        tree = Tree(baseOrigin, baseRadius, height, slimFactor, frustumMass,  myTexture, self.worldNP, self.world, self.render)
        tree.buildTrunk(50, slimFactor, frustumMass)

        #Camera Set To Look At Node1
        self.cam.setPos(0, -60, 55)
        
        self.world.setDebugNode(self.debugNP.node())


        
        self.cam.setPos(0, -60, 55)
        self.cam.lookAt(self.worldNP)
        self.world.setDebugNode(self.debugNP.node())
    
    def update(task):
        dt = globalClock.getDt()
        self.world.doPhysics(dt)
        return task.cont

    


app = MyApp()
app.run()

