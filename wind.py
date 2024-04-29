from panda3d.core import Point3, NodePath, PerlinNoise2, Vec3, ClockObject
from panda3d.core import AmbientLight, DirectionalLight, VBase4
from panda3d.bullet import BulletWorld, BulletRigidBodyNode, BulletBoxShape, BulletCylinderShape
from panda3d.bullet import ZUp
from direct.showbase.ShowBase import ShowBase
from math import pi, sin, cos, sqrt
class Wind:
    def __init__(self, direction = Vec3(1,0,0), magnitude = 1, scale=0.1, speed=0.1):
        self.direction_noise = PerlinNoise2(0.1, 0.1, 256, 0)
        self.magnitude_noise = PerlinNoise2(0.1, 0.1, 256, 1)
        self.direction = direction.normalized()
        self.magnitude = magnitude
        self.scale = scale
        self.time = 0
        self.speed = speed

    def get_wind_force(self):
        self.time += ClockObject.getGlobalClock().dt * self.speed
        angle_variation = self.direction_noise(self.time, 0) * pi 
        magnitude_variation = self.magnitude_noise(self.time, 0) * self.scale
        angle = self.direction+ Vec3(angle_variation,angle_variation,angle_variation)
        # direction = Vec3(sin(angle), cos(angle), 0)
        magnitude = max(0, self.magnitude + magnitude_variation)
        return angle * magnitude

    def get_drag_force(self, node):
        air_density = 1.225
        drag_coefficient = 0.82
        cross_sectional_area = pi * 0.5**2 #crude estimate
        velocity = node.getLinearVelocity()
        relative_velocity = self.get_wind_force() - velocity
        relative_speed = relative_velocity.length()
        drag_force_magnitude = 0.5 * air_density * relative_speed**2 * drag_coefficient * cross_sectional_area
        return -relative_velocity.normalized() * drag_force_magnitude
        