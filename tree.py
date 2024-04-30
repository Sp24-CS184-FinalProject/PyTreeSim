import pybullet as p
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
from panda3d.bullet import BulletConeTwistConstraint
from panda3d.bullet import BulletDebugNode
from panda3d.core import AmbientLight
from panda3d.core import DirectionalLight
from panda3d.core import Vec3
from panda3d.core import Vec4
from panda3d.core import Point3
from panda3d.core import TransformState
from panda3d.core import BitMask32


## Returns a RigidBodyNode Made Up Of all the Conical Frustums In The Tree 
class Tree:
    
    def __init__(self, baseOrigin, baseRadius, height, slimFactor, frustumMass, texture,  worldNP, world,  render):
        self.baseOrigin = baseOrigin
        self.baseRadius = baseRadius
        self.height = height 
        #self.orientation = orientation
        self.texture = texture
        self.baseFrustum = None
        #self.ground = ground
        self.worldNP = worldNP # Pointer to the bullet physics engine world
        self.world = world
        self.render = render # pointer to the graphics engine

    # for now, tree structure is built with frustums of equal height and mass
    def buildTrunk(self, numFrustums, slimFactor, frustumMass):
        
        frusHeight = self.height / numFrustums
        # create base Frustum 
        topPos = np.array([self.baseOrigin[0], self.baseOrigin[1], self.baseOrigin[2] + frusHeight])
        frus = Frustum(self.baseOrigin, self.baseRadius, topPos, self.baseRadius * slimFactor, None, None, None, None)
        currNP, currVisualNP = self.buildFrustum(frus, frustumMass)

        currRadius = self.baseRadius * slimFactor
        

        # Setup Constraints for all joints
        frame1 = TransformState.makeHpr(Vec3(0, 0, 0)) # Positions Are Already in place, so is orientation so we can set Haw:0, Pitch:0  Roll: 0
        frame2 = TransformState.makeHpr(Vec3(0, 0, 0))
        #Set Allowable Cone Twistt Angles 
        swing1 = 60 # degrees 
        swing2 = 36 # degrees
        twist = 120 # degrees
        damping = .9

        i = 1
        while i < numFrustums:
            nextTopPos = np.array([topPos[0], topPos[1], topPos[2] + frusHeight])

            nextFrus = Frustum(topPos, currRadius, nextTopPos, currRadius * slimFactor, None, None, None, None)

            nextNP, nextVisualNP = self.buildFrustum(nextFrus, frustumMass * slimFactor)

            self.addConstraint(currNP, nextNP, swing1, swing2, twist, damping)

            currNP, currVisualNP = nextNP, nextVisualNP
            topPos = nextTopPos
            currRadius = currRadius * slimFactor
            frustumMass = frustumMass * slimFactor
            i += 1
        


    
    # Creates the Physical And Graphical NodePaths For a Frustum Object and attaches them to the correct worldNodes, Two Return Values 
    def buildFrustum(self, frustum, frustumMass):
        geom = frustum.generateMesh(20)
        mesh = BulletTriangleMesh()
        mesh.addGeom(geom)
        shape = BulletTriangleMeshShape(mesh, dynamic=True)
        
        frusNP = self.worldNP.attachNewNode(BulletRigidBodyNode('Frustum'))
        frusNP.node().setMass(frustumMass)
        frusNP.node().addShape(shape)

        frusNP.setCollideMask(BitMask32.allOn())
        #self.boxNP.node().setDeactivationEnabled(False)
        self.world.attachRigidBody(frusNP.node())

        # Visual Node Created With Texture And Attached to Renderer 
        node = GeomNode('FRUSTUMNODE')
        node.addGeom(geom)
        visualFrusNP = self.render.attachNewNode(node)
        visualFrusNP.setTexture(self.texture)
        visualFrusNP.reparentTo(frusNP)

        return frusNP, visualFrusNP
    
    # Adds a conetwist constraint between two frustums, assumes each is next to one another in world space
    def addConstraint(self, frus1NP, frus2NP, swing1, swing2, twist, damping):
        #Setup ConeTwistConstraint Between Two Frustums 
        frame1 = TransformState.makeHpr(Vec3(0, 0, 0)) # Positions Are Already in place, so is orientation so we can set Haw:0, Pitch:0  Roll: 0
        frame2 = TransformState.makeHpr(Vec3(0, 0, 0))

        # Add Constraint With Limits to Physics Engine
        cs = BulletConeTwistConstraint(frus1NP.node(), frus2NP.node(), frame1, frame2)
        cs.setDebugDrawSize(2.0)
        cs.setLimit(swing1, swing2, twist)
        cs.setDamping(damping)
        self.world.attachConstraint(cs)

    # add_branch(parentBranch, numFrustums, slimFactor, frustumMass, length, orientation (angle from parent))
