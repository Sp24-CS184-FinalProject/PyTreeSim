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
from panda3d.core import LQuaternionf
from panda3d.core import Vec4
from panda3d.core import Point3
from panda3d.core import TransformState
from panda3d.core import BitMask32
import random


## Returns a RigidBodyNode Made Up Of all the Conical Frustums In The Tree 
class Tree:
    
    def __init__(self, type, height, baseOrigin, baseRadius, texture, worldNP, world, render):
        self.type = type
        self.height = height
        self.baseOrigin = baseOrigin
        self.baseRadius = baseRadius
        self.texture = texture
        self.worldNP = worldNP # Pointer to the bullet physics engine world
        self.world = world
        self.render = render # pointer to the graphics engine
        self.trunk = None
        self.numTrunkFrustums = 50
        self.slimFactor = .98
        self.frustumMass = 10.0
        self.frustumNPNodes = []

        # TODO: organize these types in a way that makes sense
        # different types have different branch patterns
        if type == 'original':
            self.levels = 3
            self.numChildren = 24
            self.childrenFalloff = 12
            self.minBranchThickness = 0.9
            self.maxBranchThickness = 1.1
            self.minBranchLength = 4.0
            self.maxBranchLength = 12.0
        elif type == 'dense':
            self.levels = 4
            self.numChildren = 27
            self.childrenFalloff = 3
            self.minBranchThickness = 0.7
            self.maxBranchThickness = 0.9
            self.minBranchLength = 4.0
            self.maxBranchLength = 12.0
        elif type == 'sparse':
            self.levels = 3
            self.numChildren = 10
            self.childrenFalloff = 5
            self.minBranchThickness = 0.9
            self.maxBranchThickness = 1.1
            self.minBranchLength = 4.0
            self.maxBranchLength = 12.0
        else:
            self.levels = 1
            self.numChildren = 0
            self.childrenFalloff = 0
            self.minBranchThickness = 0
            self.maxBranchThickness = 0
            self.minBranchLength = 0
            self.maxBranchLength = 0
        self.build(self.levels, None, self.numChildren, self.minBranchThickness, self.maxBranchThickness, self.minBranchLength, self.maxBranchLength)
        self.getPhysicsNodes()[0].setMass(0) # Give Frustum Attached to Ground Infinite Mass to Represent the fact it is planted 

    def getPhysicsNodes(self):
        return self.frustumNPNodes 

    def build(self, level, parent, numChildren, minBranchThickness, maxBranchThickness, minBranchLength, maxBranchLength):
        if level == 0:
            return
        if parent == None:
            current = self.buildTrunk()
        else:
            #startRelative = random.uniform(0.9, 1.6)
            parentFrustums = len(parent.frustums)
            startRelative = (int) (random.uniform(parentFrustums / 4, parentFrustums))
            length = random.uniform(minBranchLength, maxBranchLength)
            baseRadius = random.uniform(minBranchThickness / 2, maxBranchThickness / 2)
            current = self.addBranch(parent, startRelative, length, baseRadius, 10, self.slimFactor, 1.0)
        
        # have different types -- ex: one less child per branch for every level, diff frustum type / slimming ranges
        for i in range(numChildren):
            self.build(level - 1, current, numChildren // self.childrenFalloff, minBranchThickness / 1.4, maxBranchThickness / 1.4, minBranchLength / 2, maxBranchLength / 2)

    def buildTrunk(self):
        self.trunk = Branch(True, self.height, self.baseOrigin, self.baseRadius, self.numTrunkFrustums, self.slimFactor,
                        self.frustumMass, self.texture, self.worldNP, self.world, self.render)
        self.trunk.build()
        self.frustumNPNodes.extend(self.trunk.getPhysicsNodes())
        return self.trunk
    
    def addBranch(self, parent, startRelative, length, baseRadius, numFrustums, slimFactor, frustumMass):
        #startPos = parent.startPos + startRelative * parent.length * parent.direction
        jointFrustum = parent.frustums[startRelative]
        startPos = jointFrustum[0].baseOrigin
        branch = Branch(False, length, startPos, baseRadius, numFrustums, slimFactor,
                        frustumMass, self.texture, self.worldNP, self.world, self.render)
        branch.build()
        swing1 = 60 # degrees 
        swing2 = 36 # degrees
        twist = 120 # degrees
        damping = .9
        parent.addConstraint(jointFrustum[1], branch.baseFrustumNP, swing1, swing2, twist, damping)
        parent.children.append(branch)
        return branch

class Branch:
    
    def __init__(self, trunk, length, startPos, baseRadius, numFrustums, slimFactor, frustumMass, texture, worldNP, world, render):
        self.length = length
        self.startPos = startPos
        self.frustumNPNodes = []
        if trunk:
            self.direction = np.array([0, 0, 1])
        else:
            branchVector = np.array([random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-0.1, 1)])
            self.direction = branchVector / np.linalg.norm(branchVector)
        self.endPos = startPos + self.length * self.direction
        self.baseRadius = baseRadius
        self.numFrustums = numFrustums
        self.slimFactor = slimFactor
        self.frustumMass = frustumMass
        self.texture = texture
        self.baseFrustum = None
        self.worldNP = worldNP # Pointer to the bullet physics engine world
        self.world = world
        self.render = render # pointer to the graphics engine
        self.frustums = []
        self.children = []

    def getPhysicsNodes(self):
        return self.frustumNPNodes 

    def build(self):
        self.frusLength = self.length / self.numFrustums
        frustumMass = self.frustumMass
        currentPos = list(self.endPos)
        frus = Frustum(self.startPos, self.baseRadius, currentPos, self.baseRadius * self.slimFactor, None, None, None, None)
        currNP, currVisualNP = self.buildFrustum(frus, frustumMass)
        self.baseFrustumNP = currNP
        self.frustums.append([frus, currNP])
        currRadius = self.baseRadius * self.slimFactor
        
        
        #Set Allowable Cone Twistt Angles 
        swing1 = 10 # degrees 
        swing2 = 10 # degrees
        twist = 15 # degrees
        damping = .2
        softness = .8
        bias = .3
        relaxation = .3

        i = 1
        while i < self.numFrustums:
            nextPos = currentPos + self.frusLength * self.direction

            nextFrus = Frustum(currentPos, currRadius, nextPos, currRadius * self.slimFactor, None, None, None, None)
            nextNP, nextVisualNP = self.buildFrustum(nextFrus, frustumMass * self.slimFactor)
            self.frustums.append([nextFrus, nextNP])

            self.addConstraint(currNP, nextNP, swing1, swing2, twist, damping, softness, bias, relaxation)
            currNP, currVisualNP = nextNP, nextVisualNP
            currentPos = nextPos
            currRadius = currRadius * self.slimFactor
            frustumMass = frustumMass * self.slimFactor
            i += 1
        
    # Creates the Physical And Graphical NodePaths For a Frustum Object and attaches them to the correct worldNodes, Two Return Values 
    def buildFrustum(self, frustum, frustumMass):
        geom = frustum.generateMesh(10)
        mesh = BulletTriangleMesh()
        mesh.addGeom(geom)
        shape = BulletTriangleMeshShape(mesh, dynamic=False)
        
        frusNP = self.worldNP.attachNewNode(BulletRigidBodyNode('Frustum'))
        frusNP.node().setMass(frustumMass)
        frusNP.node().addShape(shape)
        frusNP.node().setDeactivationEnabled(False)
        self.frustumNPNodes.append(frusNP.node())

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
    def addConstraint(self, frus1NP, frus2NP, swing1, swing2, twist, damping, softness , bias, relaxation):
        #Setup ConeTwistConstraint Between Two Frustums 
        # Input Pos Should be position of joint in world space 
        center2Pos = self.frusLength / 2
        # Calculate Joint Position Relative to frus1NP Center Of Mass 
        pos1 = Point3(0, 0, center2Pos )
        # Calculate Joint Position Relative to frus2NP Center Of Mass 
        pos2 = Point3(0, 0, -1 * center2Pos)
        origin = Point3(0, 0, 0)
        frame1 = TransformState.makePosHpr(origin, Vec3(0, 0, -90)) #
        frame2 = TransformState.makePosHpr(origin,Vec3(0, 0, -90))

        # Add Cone Constraint With Limits to Physics Engine
        cs = BulletConeTwistConstraint(frus1NP.node(), frus2NP.node(), frame1, frame2)
        cs.setDebugDrawSize(10.0)
        cs.setLimit(swing1=swing1, swing2=swing2, twist=twist, softness=softness , bias=bias, relaxation=relaxation)
        cs.setDamping(damping)
        cs.setFixThreshold(1.0)
        cs.setMotorTargetInConstraintSpace(LQuaternionf(0, 0, 0, 0))
        cs.setMaxMotorImpulse(8.0)
        cs.enableMotor(True)
        self.world.attach(cs)

        # Add Piston Constraint To Limit Weird Translation 
        #slider = BulletSliderConstraint(frus1NP.node(), frus2NP.node(), frame1, frame2, True)
        #slider.setLowerLinearLimit(0.0001)
        #slider.setUpperLinearLimit(0.001)
        #self.world.attach(slider)
