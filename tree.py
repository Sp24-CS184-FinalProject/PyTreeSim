import pybullet as p
import pybullet_data
from frustum import *

class Tree:
    
    def __init__(self, baseOrigin, baseRadius, height, orientation, texture, ground):
        self.baseOrigin = baseOrigin
        self.baseRadius = baseRadius
        self.height = height 
        self.orientation = orientation
        self.texture = texture
        self.baseFrustum = None
        self.ground = ground

    # for now, tree structure is built with frustums of equal height and mass
    def buildTrunk(self, numFrustums, slimFactor, frustumMass):
        linkMasses = [0.1]
        linkPositions = [[0, 0, 0]]
        linkOrientations = [[0, 0, 0, 1]]
        linkInertialFramePositions = [[0, 0, 0]]
        linkInertialFrameOrientations = [[0, 0, 0, 1]]
        axis = [[0, 0, 1]]
        frustumHeight = self.height / numFrustums
        bottomOrigin = list(self.baseOrigin)
        bottomRadius = self.baseRadius
        texUid = p.loadTexture(self.texture)

        # create frustums
        prev = None
        for i in range(numFrustums):
            topOrigin = np.array(bottomOrigin[:])
            topOrigin[2] += frustumHeight
            topRadius = bottomRadius * slimFactor
            print('bottom origin: ', bottomOrigin)
            print('top origin: ', topOrigin)
            frus = Frustum(bottomOrigin, bottomRadius, topOrigin, topRadius, 1.0, prev, None, i + 1)
            frus.generateMesh(100)

            if prev:
                prev.child = frus
            else:
                self.baseFrustum = frus
            prev = frus
            bottomOrigin = topOrigin
            bottomRadius = topRadius

        current = self.baseFrustum
        while current:
            if current.child is None:
                trunkId = p.createMultiBody(0.01, current.CollisionId, current.VisualId)
            else:
                if current.parent is None:
                    parent_indices = [0]
                    mass = frustumMass
                else:
                    parent_indices = [current.parent.index]
                    mass = 0.01

                # ideally the joint is type spherical but that causes base to disappear
                trunkId = p.createMultiBody(mass,
                                            current.CollisionId,
                                            current.VisualId,
                                            linkMasses=linkMasses,
                                            linkCollisionShapeIndices=[current.child.CollisionId],
                                            linkVisualShapeIndices=[current.child.VisualId],
                                            linkPositions=linkPositions,
                                            linkOrientations=linkOrientations,
                                            linkInertialFramePositions=linkInertialFramePositions,
                                            linkInertialFrameOrientations=linkInertialFrameOrientations,
                                            linkParentIndices=parent_indices,
                                            linkJointTypes=[p.JOINT_REVOLUTE],
                                            linkJointAxis=axis)
            
            p.changeVisualShape(trunkId, -1, textureUniqueId=texUid)
            p.changeDynamics(trunkId,
                            current.index,
                            spinningFriction=0.001,
                            rollingFriction=0.001,
                            linearDamping=0.0)
            for joint in range(p.getNumJoints(trunkId)):
                p.setJointMotorControl2(trunkId, joint, p.VELOCITY_CONTROL, targetVelocity=1, force=10)
            
            current = current.child

    # add_branch(parentBranch, numFrustums, slimFactor, frustumMass, length, orientation (angle from parent))
