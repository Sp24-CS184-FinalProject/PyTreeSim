from panda3d.core import Point3, NodePath, PerlinNoise2, Vec3, ClockObject
from panda3d.core import AmbientLight, DirectionalLight, VBase4
from panda3d.bullet import BulletWorld, BulletRigidBodyNode, BulletBoxShape, BulletCylinderShape
from panda3d.bullet import ZUp
from direct.showbase.ShowBase import ShowBase
from math import pi, sin, cos, sqrt
class Wind:
    def __init__(self, scale=1.0, speed=1.0):
        self.direction_noise = PerlinNoise2(0.1, 0.1, 256, 0)
        self.magnitude_noise = PerlinNoise2(0.1, 0.1, 256, 1)
        self.time = 0
        self.scale = scale
        self.speed = speed

    def get_wind_force(self):
        self.time += ClockObject.getGlobalClock().dt * self.speed
        angle = self.direction_noise(self.time, 1) * 2 * pi
        magnitude = (self.magnitude_noise(self.time, 1) + 1) / 2 * self.scale
        return Vec3(sin(angle) * magnitude, cos(angle) * magnitude, 0)

    def get_drag_force(self, node):
        dt = ClockObject.getGlobalClock().dt
        self.physics_world.doPhysics(dt)
        air_density = 1.225
        drag_coefficient = 0.82
        cross_sectional_area = pi * 0.5**2 #crude estimate
        velocity = node.getLinearVelocity()
        relative_velocity = self.get_wind_force() - velocity
        relative_speed = relative_velocity.length()
        drag_force_magnitude = 0.5 * air_density * relative_speed**2 * drag_coefficient * cross_sectional_area
        return -relative_velocity.normalized() * drag_force_magnitude
        

class App(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        
        self.physics_world = BulletWorld()
        self.physics_world.setGravity(0, 0, -9.81)
        
        self.scene = self.loader.loadModel("models/environment")
        self.scene.reparentTo(self.render)
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, 0)
        
        #todo
        # trunk_shape = BulletCylinderShape(0.5, 2, ZUp)
        # trunk_node = BulletRigidBodyNode('Trunk')
        # trunk_node.addShape(trunk_shape)
        # trunk_node.setMass(10)
        # trunk_np = self.render.attachNewNode(trunk_node)
        # trunk_np.setPos(0, 0, 1)
        # self.physics_world.attachRigidBody(trunk_node)
                
        # Lighting
        alight = AmbientLight('ambient')
        alight.setColor(VBase4(0.5, 0.5, 0.5, 1))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)
        
        dlight = DirectionalLight('directional')
        dlight.setColor(VBase4(0.8, 0.8, 0.8, 1))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(0, -60, 0)
        self.render.setLight(dlnp)
        
        self.wind_simulator = Wind(scale=50, speed=0.2)
        self.taskMgr.add(self.update, "updateWorld")
    
    def update(self, task):
          
        wind_force = self.wind_simulator.get_wind_force()
        for node in self.physics_world.getRigidBodies():
            drag_force = self.wind_simulator.get_drag_force(node)
            node.applyCentralForce(wind_force + drag_force)
        return task.cont


app = App()
app.run()
