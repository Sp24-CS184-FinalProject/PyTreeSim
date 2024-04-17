import pybullet as p
import time
import math
import pybullet_data
from frustum import *

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#don't create a ground plane, to allow for gaps etc
p.resetSimulation()
#p.createCollisionShape(p.GEOM_PLANE)
#p.createMultiBody(0,0)
#p.resetDebugVisualizerCamera(5,75,-26,[0,0,1]);
p.resetDebugVisualizerCamera(10, -346, -16, [0, 0, 0])

p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)



#a few different ways to create a mesh:


#convex mesh from obj
baseOrigin = np.array([0.2, 0.2, .01])
baseRadius = 3.0
topOrigin = np.array([0.2, 0.2, 3.01])
topRadius = 2.0

orientation = 1.0
frus = Frustum(baseOrigin, baseRadius, topOrigin, topRadius, orientation, None, None, -1)
frus.generateMesh(20)
frustId = frus.CollisionId
frustVisualId = frus.VisualId
texUid = p.loadTexture("Pinewood_Bark_DIFF.png")

boxHalfLength = 0.5
boxHalfWidth = 2.5
boxHalfHeight = 0.1
segmentLength = 5


mass = 1
visualShapeId = -1

segmentStart = 0

baseOrientation = p.getQuaternionFromEuler([math.pi / 2., 0, math.pi / 2.])


for i in range(1):
  
  width = 4
  for j in range(1):
    bodyUid = p.createMultiBody(baseMass=0,
                      baseCollisionShapeIndex=frustId,
                      baseVisualShapeIndex=frustVisualId,
                      basePosition=[segmentStart, 0.5 * (i % 2) + j - width / 2., 0])
  segmentStart = segmentStart - 1

p.changeVisualShape(bodyUid, -1, textureUniqueId=texUid)



p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
while (1):
  camData = p.getDebugVisualizerCamera()
  viewMat = camData[2]
  projMat = camData[3]
  p.getCameraImage(256,
                   256,
                   viewMatrix=viewMat,
                   projectionMatrix=projMat,
                   renderer=p.ER_BULLET_HARDWARE_OPENGL)
  keys = p.getKeyboardEvents()
  p.stepSimulation()
  #print(keys)
  time.sleep(0.01)