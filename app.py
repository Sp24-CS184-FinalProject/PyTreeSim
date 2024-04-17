import pybullet as p
import time
import pybullet_data
from frustum import Frustum
import numpy as np
import math


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

#Add a simple equilateral triangle with specified thickness
def addTriangle(side_length, thickness, position, orientation, color=[1, 0, 0, 1], mass=1):
   
    height = (math.sqrt(3)/2) * side_length

    vertices = [
        [0, -height / 3, 0],
        [side_length / 2, 2 * height / 3, 0],
        [-side_length / 2, 2 * height / 3, 0]
    ]
    vertices += [[v[0], v[1], thickness] for v in vertices]
    

    indices = [
        0, 1, 2, # Bottom triangle
        3, 5, 4, # Top triangle
        0, 3, 4, 0, 4, 1, # Side 1
        1, 4, 5, 1, 5, 2, # Side 2
        2, 5, 3, 2, 3, 0  # Side 3
    ]

    quaternion = p.getQuaternionFromEuler(orientation)
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_MESH, vertices=vertices, indices=indices, rgbaColor=color)
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_MESH, vertices=vertices, indices=indices)

    triangle_id = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_shape_id, baseVisualShapeIndex=visual_shape_id, basePosition=position, baseOrientation=quaternion)
    
    return triangle_id
    
    
    
def addFrustum(top_radius, bottom_radius, height, position, orientation =[0,0,0], color = [0,1,0], num_sides = 100):
    frustum = Frustum(baseOrigin=np.array(position),baseRadius=bottom_radius,topOrigin=np.array([position[0],position[1],position[2]+height]),topRadius=top_radius,orientation=orientation)
    frustum.generateMesh(num_sides)
    
    meshScale = [1,1,1]
    quaternion = p.getQuaternionFromEuler(orientation)
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                          vertices=frustum.vertices,
                                          indices=frustum.indices,
                                          meshScale=meshScale,
                                          rgbaColor=color,
                                          uvs = frustum.uvs,
                                          normals = frustum.normals)
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                                vertices=frustum.vertices,
                                                indices=frustum.indices,
                                                meshScale=meshScale)
    frustum_id = p.createMultiBody(baseMass=1,
                                   baseCollisionShapeIndex=collision_shape_id,
                                   baseVisualShapeIndex=visual_shape_id,
                                   basePosition=position,
                                   baseOrientation=quaternion)
    return frustum_id


physicsClient = p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#initial scene stuff
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

#placing objects

frustumId = addFrustum(top_radius=2, bottom_radius=1.0, height=2.0, position=[0,0,0.1], orientation=[0, 0, 0], color=[0, 1, 0, 1])
# triangle = addTriangle(2,0.5, [0,0,0.1],[0,0,0],mass=10)
# triangle = addTriangle(2,0.5, [0,0,0.1],[0,0,0],mass=10)

# Simulation loop
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()

