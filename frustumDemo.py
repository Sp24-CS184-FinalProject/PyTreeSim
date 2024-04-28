
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
from panda3d.core import AmbientLight
from panda3d.core import DirectionalLight
from panda3d.core import Vec3
from panda3d.core import Vec4
from panda3d.core import Point3
from panda3d.core import TransformState
from panda3d.core import BitMask32


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
        self.frusNP.node().setMass(10.0)
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
        self.frus1NP.node().setMass(10.0)
        self.frus1NP.node().addShape(shape)
        self.frus1NP.setCollideMask(BitMask32.allOn())

        self.world.attachRigidBody(self.frus1NP.node())

        # Second Visual Node Created And Attached To First Visual Node 
        node1 = GeomNode('FRUSTUMNODE')
        node1.addGeom(geom1)
        self.nodePath1 = self.nodePath.attachNewNode(node1)
        self.nodePath1.setTexture(myTexture)
        self.nodePath1.reparentTo(self.frus1NP)

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
    
    def update(task):
        dt = globalClock.getDt()
        self.world.doPhysics(dt)
        return task.cont

    


app = MyApp()
app.run()
class Game(DirectObject):

  def __init__(self):
    base.setBackgroundColor(0.1, 0.1, 0.8, 1)
    base.setFrameRateMeter(True)

    base.cam.setPos(0, -20, 4)
    base.cam.lookAt(0, 0, 0)

    # Light
    alight = AmbientLight('ambientLight')
    alight.setColor(Vec4(0.5, 0.5, 0.5, 1))
    alightNP = render.attachNewNode(alight)

    dlight = DirectionalLight('directionalLight')
    dlight.setDirection(Vec3(1, 1, -1))
    dlight.setColor(Vec4(0.7, 0.7, 0.7, 1))
    dlightNP = render.attachNewNode(dlight)

    render.clearLight()
    render.setLight(alightNP)
    render.setLight(dlightNP)

    # Input
    self.accept('escape', self.doExit)
    self.accept('r', self.doReset)
    self.accept('f1', self.toggleWireframe)
    self.accept('f2', self.toggleTexture)
    self.accept('f3', self.toggleDebug)
    self.accept('f5', self.doScreenshot)

    inputState.watchWithModifiers('forward', 'w')
    inputState.watchWithModifiers('left', 'a')
    inputState.watchWithModifiers('reverse', 's')
    inputState.watchWithModifiers('right', 'd')
    inputState.watchWithModifiers('turnLeft', 'q')
    inputState.watchWithModifiers('turnRight', 'e')

    # Task
    taskMgr.add(self.update, 'updateWorld')

    # Physics
    self.setup()

  # _____HANDLER_____

  def doExit(self):
    self.cleanup()
    sys.exit(1)

  def doReset(self):
    self.cleanup()
    self.setup()

  def toggleWireframe(self):
    base.toggleWireframe()

  def toggleTexture(self):
    base.toggleTexture()

  def toggleDebug(self):
    if self.debugNP.isHidden():
      self.debugNP.show()
    else:
      self.debugNP.hide()

  def doScreenshot(self):
    base.screenshot('Bullet')

  # ____TASK___

  def processInput(self, dt):
    force = Vec3(0, 0, 0)
    torque = Vec3(0, 0, 0)

    if inputState.isSet('forward'): force.setY( 1.0)
    if inputState.isSet('reverse'): force.setY(-1.0)
    if inputState.isSet('left'):    force.setX(-1.0)
    if inputState.isSet('right'):   force.setX( 1.0)
    if inputState.isSet('turnLeft'):  torque.setZ( 1.0)
    if inputState.isSet('turnRight'): torque.setZ(-1.0)

    force *= 30.0
    torque *= 10.0

    force = render.getRelativeVector(self.boxNP, force)
    torque = render.getRelativeVector(self.boxNP, torque)

    self.boxNP.node().setActive(True)
    self.boxNP.node().applyCentralForce(force)
    self.boxNP.node().applyTorque(torque)

  def update(self, task):
    dt = globalClock.getDt()

    self.processInput(dt)
    #self.world.doPhysics(dt)
    self.world.doPhysics(dt, 5, 1.0/180.0)

    return task.cont

  def cleanup(self):
    self.world.removeRigidBody(self.groundNP.node())
    self.world.removeRigidBody(self.boxNP.node())
    self.world = None

    self.debugNP = None
    self.groundNP = None
    self.boxNP = None

    self.worldNP.removeNode()

  def setup(self):
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

    # Ground (static)
    shape = BulletPlaneShape(Vec3(0, 0, 1), 1)

    self.groundNP = self.worldNP.attachNewNode(BulletRigidBodyNode('Ground'))
    self.groundNP.node().addShape(shape)
    self.groundNP.setPos(0, 0, -2)
    self.groundNP.setCollideMask(BitMask32.allOn())

    self.world.attachRigidBody(self.groundNP.node())

    # Box (dynamic)
    shape = BulletBoxShape(Vec3(0.5, 0.5, 0.5))

    self.boxNP = self.worldNP.attachNewNode(BulletRigidBodyNode('Box'))
    self.boxNP.node().setMass(1.0)
    self.boxNP.node().addShape(shape)
    self.boxNP.setPos(0, 0, 2)
    #self.boxNP.setScale(2, 1, 0.5)
    self.boxNP.setCollideMask(BitMask32.allOn())
    #self.boxNP.node().setDeactivationEnabled(False)

    self.world.attachRigidBody(self.boxNP.node())

    self.visualNP = loader.loadModel('models/box.egg')
    self.visualNP.clearModelNodes()
    visualNP.reparentTo(self.boxNP)
    
    

    # Bullet nodes should survive a flatten operation!
    #self.worldNP.flattenStrong()
    #render.ls()

game = Game()
run()
