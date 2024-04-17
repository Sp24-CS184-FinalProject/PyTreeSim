import pybullet as p
import time
import math
import pybullet_data
from tree import *

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.createCollisionShape(p.GEOM_PLANE)
groundId = p.createMultiBody(0, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

baseOrigin = np.array([0.2, 0.2, .01])
baseRadius = 0.5
height = 5.0
orientation = 1.0
texture = "Pinewood_Bark_DIFF.png"
tree = Tree(baseOrigin, baseRadius, height, orientation, texture, groundId)
tree.buildTrunk(50, 0.95, 20.0)

p.setGravity(10, 0, 0)
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
  time.sleep(0.01)

#p.setGravity(0, 0, -10)
# p.setRealTimeSimulation(1)
# while (1):
#   keys = p.getKeyboardEvents()
#   print(keys)

#   time.sleep(0.01)
