import pybullet as p
import time
import pybullet_data


def addCube(halfExtents, startPosition, startOrientation, color=[1, 0, 0, 1], mass=1):
    #use this template for triangle and frustums
    
    quaternion = p.getQuaternionFromEuler(startOrientation)
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX, rgbaColor=color, halfExtents=halfExtents)
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=halfExtents)
    
    cube_id = p.createMultiBody(baseMass=mass, 
                                baseCollisionShapeIndex=collision_shape_id, 
                                baseVisualShapeIndex=visual_shape_id, 
                                basePosition=startPosition, 
                                baseOrientation=quaternion)
    return cube_id

def createFrustumMesh():
    pass

def addFrustum(top_radius, bottom_radius, height, position, orientation, color):
    vertices, indices = createFrustumMesh()
    

physicsClient = p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#initial scene stuff
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

#placing objects
cubeId = addCube(halfExtents=[0.5, 0.5, 0.5], startPosition=[0, 0, 1], startOrientation=[0, 0, 0])
cubeId = addCube(halfExtents=[0.3, 0.3, 0.3], startPosition=[0, 2, 1], startOrientation=[0, 0, 0])

# Simulation loop
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()

