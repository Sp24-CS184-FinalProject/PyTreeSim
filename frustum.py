import pybullet as p
import numpy as np
import time
import pybullet_data
import math


class Frustum:
    
    def __init__(self, baseOrigin, baseRadius, topOrigin, topRadius, orientation):
        self.baseOrigin = baseOrigin
        self.baseRadius = baseRadius
        self.topOrigin = topOrigin
        self.topRadius = topRadius
        self.height = np.linalg.norm(topOrigin - baseOrigin)
        self.orientation = orientation
        self.vertices = []
        self.uvs = []
        self.normals = []
        self.indices = []
        self.triangles = [] # list of three element lists, where each element is a vertex of a triangle 
        self.CollisionId = -1
        self.VisualId = -1


    def generateMesh(self, numSides):
        self.generateVertices(numSides)
        self.generateIndices()
        # Add the origin of the top and bottom circles to vertices to render the tops and bottoms 
        self.vertices.append(self.baseOrigin) # index len(vertices) - 2
        self.vertices.append(self.topOrigin)   # index len(vertices) - 1
        
        for i in range(len(self.vertices) - 2):
            if (i % 3 == 1): # connect top circle vertices to top origin 
                self.indices.append(i)
                self.indices.append(len(self.vertices) - 1) # index of topOrigin
                if (i + 3 >= len(self.vertices) - 2): # check if vertex to right exists
                    self.indices.append(1) # first top vertex
                else:
                    self.indices.append(i + 3)
            elif (i % 3 == 0): # connect bottom circle vertices to bottom circles origin
                self.indices.append(i)
                if (i + 3 >= len(self.vertices) - 2): # check if vertex to right exists
                    self.indices.append(0) # first top vertex
                else:
                    self.indices.append(i + 3)
                self.indices.append(len(self.vertices) - 2) # index of bottom circle 
                    
        self.generateUVCoords()
        self.generateNormals()
        # self.CollisionId =  p.createCollisionShape(p.GEOM_MESH, vertices=self.vertices, indices=self.indices)
        # self.VisualId = p.createVisualShape(shapeType=p.GEOM_MESH,
        #                             rgbaColor=[1, 1, 1, 1],
        #                             specularColor=[0.4, .4, 0],
        #                             vertices=self.vertices,
        #                             indices=self.indices,
        #                             uvs=self.uvs,
        #                             normals=self.normals)

    # Create a Mesh For This Frustum, return shapeID
    # numSides determines how many vertices to create ( I.e. the level num of triangles in mesh)
    def generateVertices(self, numSides):
        vertices = []
        for i in range(numSides):
            angle =  i / numSides * 2.0 * math.pi 
            # add base circle vertex
            baseX = self.baseOrigin[0] + self.baseRadius * math.cos(angle)
            baseY = self.baseOrigin[1] + self.baseRadius * math.sin(angle)
            baseVert = np.array([baseX, baseY, self.baseOrigin[2]])
            vertices.append(baseVert)
            # add top circle vertex
            topX = self.topOrigin[0] + self.topRadius * math.cos(angle)
            topY = self.topOrigin[1] + self.topRadius * math.sin(angle)
            topVert = np.array([topX, topY, self.topOrigin[2]])
            vertices.append(topVert)
            # add midpoint vertex on line connecting the top and base vertices
            line = (topVert - baseVert) / np.linalg.norm(topVert - baseVert) # vector connnecting top and bottom vertices from base -> top
            #dist = np.linalg.norm(topVert - baseVert) / 2.0
            midpoint = baseVert + math.sqrt((self.baseRadius - self.topRadius) * (self.baseRadius - self.topRadius) + self.height * self.height ) / 2.0 * line
            vertices.append(midpoint)
        
        self.vertices = vertices

    # generate the indicies of the triangles in this mesh, should be a multiple of three
    # index 0 refers to the first vertex in self.vertices, a triangle is defined by three consequtive vertices 
    # each refering to a vertex in self.vertices
    # Vertices are added in the following order: bottom, top, mid, bottom , top, mid
    # where bottom is a vertex on the bottom circle, top is a vertex on the top circle, and mid is a vertex on the midpoint connecting the two
    # indices are generated in counter clock wise order
    def generateIndices(self):
        indices = []
        for i in range(len(self.vertices)):
            if (i % 3 == 0): # we are at a bottom vertex
                # add first triangle 
                indices.append(i)
                indices.append(i + 2) # mid vertex above i, guarenteed to exist
                if (i + 5 >= len(self.vertices)): # out of bounds, wrap around to mid vertex in first column.
                    indices.append(2) # mid vertex of first column
                else: 
                    indices.append(i + 5) # mid vertex up and to the right of i
                # add second triangle 
                indices.append(i)
                # Get index of vertex up and to the right
                if (i + 5 >= len(self.vertices)): # out of bounds, wrap around to mid vertex in first column.
                    indices.append(2) # mid vertex of first column
                else: 
                    indices.append(i + 5) # mid vertex up and to the right of i
                # Get index of vertex to the right else wrap around 
                if (i + 3 >= len(self.vertices)): 
                    indices.append(0)
                else: 
                    indices.append(i + 3)

            # Repeat logic for top row of triangles with a mid vertex as the base 
            if (i % 3 == 2): # we are at a middle vertex
                # add first triangle 
                indices.append(i)
                indices.append(i - 1) # top vertex above i, guarenteed to exist
                if (i + 2 >= len(self.vertices)): # out of bounds, wrap around to top vertex in first column.
                    indices.append(1) # top vertex of first column
                else: 
                    indices.append(i + 2) # mid vertex up and to the right of i
                # add second triangle 
                indices.append(i)
                # Get index of vertex up and to the right
                if (i + 2 >= len(self.vertices)): # out of bounds, wrap around to mid vertex in first column.
                    indices.append(1) # mid vertex of first column
                else: 
                    indices.append(i + 2) # mid vertex up and to the right of i
                # Get index of vertex to the right else wrap around 
                if (i + 3 >= len(self.vertices)): 
                    indices.append(2)
                else: 
                    indices.append(i + 3)  


        self.indices = indices


    # Called with buildMesh to generate the UV Texture Coords for this object
    def generateUVCoords(self):
        uvs = []
        for vertex in self.vertices:
            uvs.append(self.cartToUV(vertex))
        self.uvs = uvs
    
    def generateNormals(self):
        normals = []
        for vertex in self.vertices: 
            
            normX =  2 * vertex[0] * (self.topRadius - self.baseRadius) / self.height
            normY = 2 * vertex[1] * (self.topRadius - self.baseRadius) / self.height
            normZ =  2 * vertex[2] - self.height
            normals.append([normX, normY, normZ])

        self.normals = normals

    # Cartesian Coordinates to UV Coords (r, \theta)
    def cartToUV(self, coord):
        uv = []
        u = math.sqrt( coord[0] * coord[0] + coord[1] * coord[1])
        v = math.atan(coord[1] / coord[0])
        uv.append(u)
        uv.append(v)
        return uv