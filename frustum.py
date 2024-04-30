import pybullet as p
import numpy as np
import time
import pybullet_data
import math
from panda3d.core import GeomVertexFormat
from panda3d.core import GeomVertexData
from panda3d.core import GeomVertexWriter
from panda3d.core import GeomTriangles
from panda3d.core  import Geom
from panda3d.core import GeomPrimitive


class Frustum:
    
    def __init__(self, baseOrigin, baseRadius, topOrigin, topRadius, orientation, parent, child, index):
        self.baseOrigin = baseOrigin
        self.baseRadius = baseRadius
        self.topOrigin = topOrigin
        self.topRadius = topRadius
        self.height = np.linalg.norm(topOrigin - baseOrigin)
        self.orientation = orientation
        self.parent = parent
        self.child = child
        self.index = index
        self.vertices = []
        self.uvs = []
        self.normals = []
        self.indices = []
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
        self.reverseWindingOrder()     
        self.generateUVCoords()
        self.generateNormals()
        vformat = GeomVertexFormat.getV3n3t2()
        vdata = GeomVertexData(name='', format=vformat, usage_hint=2) # usage hint tells us the vertex data may be manipulated later
        vdata.setNumRows(len(self.vertices))
        vertexWriter = GeomVertexWriter(vdata, 'vertex')
        normalWriter = GeomVertexWriter(vdata, 'normal')
        texcoordWriter = GeomVertexWriter(vdata, 'texcoord')

        for i in range(len(self.vertices)):
            vert = self.vertices[i]
            norm = self.normals[i]
            uvs = self.uvs[i]
            vertexWriter.addData3(vert[0], vert[1], vert[2])
            normalWriter.addData3(norm[0], norm[1], norm[2])
            texcoordWriter.addData2(uvs[0], uvs[1])

        prim = GeomTriangles(Geom.UHStatic) #set indices to static 
        i = 0
        while i < len(self.indices):
            if i % 3 != 0:
                continue
            prim.add_vertices(self.indices[i], self.indices[i+1], self.indices[i+2])
            prim.closePrimitive()
            i += 3

        geom = Geom(vdata)
        geom.addPrimitive(prim)

        return geom

         

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
        
        self.vertices = vertices[:]

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


        self.indices = indices[:]

    # Messed up on winding order of triangles when doing generateIndices, Flip them here
    def reverseWindingOrder(self):

        i = 0
        while i < len(self.indices):
            indexA = self.indices[i]
            indexB = self.indices[i+1]
            indexC = self.indices[i+2]

            self.indices[i] = indexC
            self.indices[i+1] = indexB
            self.indices[i+2] = indexA
            i += 3


    # Called with buildMesh to generate the UV Texture Coords for this object
    def generateUVCoords(self):
        uvs = []
        for vertex in self.vertices:
            uvs.append(self.cartToUV(vertex))
        self.uvs = uvs[:]
    
    def generateNormals(self):
        normals = []
        for vertex in self.vertices:  # set each vertex normal to 0
            normals.append(np.array([0.0, 0.0 , 0.0]))

        i = 0
        while (i < len(self.indices)): 
            # Get Positions of Triangle Vertices
            A = self.vertices[self.indices[i]]
            B = self.vertices[self.indices[i + 1]]
            C = self.vertices[self.indices[i + 2]]

            
            n = -1 * np.cross(B - A, C - A) # calculate normal vector of given triangle
            # Consequtively add that normal vector to the normals of each individual vertex
            normals[self.indices[i]] += n
            normals[self.indices[i + 1]] += n
            normals[self.indices[i + 2]] += n
            i += 3
    
        i = 0
        while (i < len(normals)):
            normals[i] /= np.linalg.norm(normals[i]) # normalize vectors
            i += 1

        
        self.normals = normals[:]
        
        


    # Cartesian Coordinates to UV Coords (r, \theta)
    def cartToUV(self, coord):
        uv = []
        u = math.sqrt( coord[0] * coord[0] + coord[1] * coord[1])
        v = math.atan(coord[1] / coord[0])
        uv.append(u)
        uv.append(v)
        return uv