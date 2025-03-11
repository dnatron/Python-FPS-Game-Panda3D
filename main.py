from dataclasses import dataclass
from panda3d.core import Point3, Vec3, CollisionNode, CollisionRay, DirectionalLight, AmbientLight, CollisionHandlerQueue, CollisionTraverser, NodePath, WindowProperties
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.bullet import BulletWorld, BulletRigidBodyNode, BulletBoxShape, BulletSphereShape
from math import sin, cos, radians, pi
from time import time


# Data classes to encapsulate player, physics world, and physics box configuration
@dataclass
class PlayerConfig:
    position: Vec3
    scale: float
    mass: float
    friction: float
    max_ammo: int
    ammo_per_reload: int
    reload_time: float


@dataclass
class PhysicsWorldConfig:
    gravity: Vec3


@dataclass
class PhysicsBoxConfig:
    position: Vec3
    size: Vec3
    mass: float
    friction: float


class FPSApp(ShowBase):
    def __init__(self):
        super().__init__()

        # Disable the default mouse control
        self.disableMouse()

        # Setup collision system
        self.cTrav = CollisionTraverser()
        self.collision_handler = CollisionHandlerQueue()

        # Setup configurations using dataclasses
        self.player_config = PlayerConfig(
            position=Vec3(0, 0, 2), 
            scale=0.5, 
            mass=1.0, 
            friction=0.5, 
            max_ammo=1000,  # Initial ammo
            ammo_per_reload=20,  # Ammo per reload
            reload_time=2.0  # Reload time in seconds
        )
        self.physics_world_config = PhysicsWorldConfig(gravity=Vec3(0, 0, -9.81))
        self.physics_boxes_config = [
            PhysicsBoxConfig(position=Vec3(5, 0, 2), size=Vec3(0.5, 0.5, 0.5), mass=1.0, friction=0.5),
            PhysicsBoxConfig(position=Vec3(5, 5, 2), size=Vec3(0.5, 0.5, 0.5), mass=1.0, friction=0.5),
        ]

        # Setup the physics world
        self.bullet_world = BulletWorld()
        self.bullet_world.set_gravity(self.physics_world_config.gravity)
        self.setup_environment()

        # Setup the player and its physics
        self.create_player()

        # Setup the physics boxes
        for box_config in self.physics_boxes_config:
            self.create_physics_box(box_config)

        # Player shooting state and ammo tracking
        self.ammo_count = self.player_config.max_ammo  # Total ammo
        self.ammo_in_clip = 0  # Ammo in the current clip (starts at 0)
        self.reload_in_progress = False  # Whether a reload is in progress
        self.last_reload_time = 0  # Time of the last reload
        self.last_shot_time = 0  # Time of the last shot
        self.reload_key_cooldown = 0  # Cooldown for reload key press

        # Set up the reload key binding
        self.accept('r', self.reload)

        # Mouse look for camera control
        self.mouse_sensitivity = 0.2
        self.prev_mouse_pos = None
        self.camera_angle = 0.0
        self.vertical_angle = 0.0
        self.task_mgr.add(self.mouse_look, "mouse_look")
        self.speed = 5.0

        # Hide the cursor
        props = WindowProperties()
        props.setCursorHidden(True)
        self.win.requestProperties(props)

        # Left mouse button shooting (raycast to simulate shooting)
        self.accept('mouse1', self.shoot)

        # Create a collision ray for shooting
        self.shoot_ray = CollisionRay()
        self.shoot_ray.set_origin(0, 0, 0)
        self.shoot_ray.set_direction(0, 1, 0)
        self.collision_node = CollisionNode('shoot_ray')
        self.collision_node.addSolid(self.shoot_ray)
        self.collision_np = self.camera.attach_new_node(self.collision_node)
        self.cTrav.addCollider(self.collision_np, self.collision_handler)

        # Set up key bindings
        self.accept('w', self.move_forward)
        self.accept('w-up', lambda: None)
        self.accept('s', self.move_backward)
        self.accept('s-up', lambda: None)
        self.accept('a', self.move_left)
        self.accept('a-up', lambda: None)
        self.accept('d', self.move_right)
        self.accept('d-up', lambda: None)
        self.accept('space', self.jump)
        self.accept('space-up', lambda: None)

        # Add update task
        self.task_mgr.add(self.update, "update")

    def create_player(self):
        # Create the player using the config data class
        self.player = self.loader.load_model("models/box")
        self.player.set_scale(self.player_config.scale)
        self.player.set_pos(self.player_config.position)
        self.player.reparent_to(self.render)

        # Physics setup for the player (Bullet)
        self.player_physics_node = BulletRigidBodyNode('player')
        self.player_shape = BulletBoxShape(Vec3(0.5, 0.5, 0.5))  # Player box shape
        self.player_physics_node.add_shape(self.player_shape)
        self.player_physics_node.set_mass(self.player_config.mass)  # Mass of the player
        self.player_physics_node.set_friction(self.player_config.friction)
        self.player_np = self.render.attach_new_node(self.player_physics_node)
        self.player_np.set_pos(self.player_config.position)
        self.bullet_world.attach_rigid_body(self.player_physics_node)

    def create_physics_box(self, box_config: PhysicsBoxConfig):
        # Create a Bullet Box shape for the physics box using the config data class
        box = self.loader.load_model("models/box")
        box.set_scale(box_config.size)
        box.reparent_to(self.render)
        box.set_pos(box_config.position)

        box_physics_node = BulletRigidBodyNode('box')
        box_shape = BulletBoxShape(Vec3(*box_config.size))  # Box size
        box_physics_node.add_shape(box_shape)
        box_physics_node.set_mass(box_config.mass)
        box_physics_node.set_friction(box_config.friction)
        box_np = self.render.attach_new_node(box_physics_node)
        box_np.set_pos(box_config.position)

        self.bullet_world.attach_rigid_body(box_physics_node)

    def move_forward(self):
        # Move in the direction the camera is facing
        self.player_np.setPos(self.player_np.getPos() + 
                            Vec3(self.camera.getMat().getRow3(1) * self.speed * globalClock.getDt()))

    def move_backward(self):
        # Move opposite to the direction the camera is facing
        self.player_np.setPos(self.player_np.getPos() - 
                            Vec3(self.camera.getMat().getRow3(1) * self.speed * globalClock.getDt()))

    def move_left(self):
        # Move perpendicular to the camera direction (left)
        self.player_np.setPos(self.player_np.getPos() - 
                            Vec3(self.camera.getMat().getRow3(0) * self.speed * globalClock.getDt()))

    def move_right(self):
        # Move perpendicular to the camera direction (right)
        self.player_np.setPos(self.player_np.getPos() + 
                            Vec3(self.camera.getMat().getRow3(0) * self.speed * globalClock.getDt()))

    def jump(self):
        # Apply force to simulate a jump
        if self.player_physics_node:
            self.player_physics_node.set_linear_velocity(Vec3(0, 0, 10))  # Apply upward force

    def shoot(self):
        if self.reload_in_progress:
            print("Reloading... Please wait.")
            return
        
        # Check if we have ammo to shoot
        if self.ammo_in_clip > 0:
            # Get the direction in which the player is looking
            direction = self.camera.get_hpr()[0]  # yaw (horizontal rotation)
            radian = radians(direction)

            # Set the collision ray direction based on where the camera is looking
            self.shoot_ray.set_origin(self.camera.get_pos())
            self.shoot_ray.set_direction(Vec3(sin(radian), cos(radian), 0))

            # Perform a collision check (raycast)
            self.collision_handler.clearEntries()
            self.cTrav.traverse(self.render)

            # If there is a collision, handle the hit
            if self.collision_handler.getNumEntries() > 0:
                hit_entry = self.collision_handler.getEntry(0)
                hit_node_path = hit_entry.getIntoNodePath()
                hit_point = hit_entry.getSurfacePoint(self.render)
                print(f"Hit point: {hit_point}")
                self.create_hit_effect(hit_point)

            # Decrease ammo after shooting
            self.ammo_in_clip -= 1
            print(f"Ammo left: {self.ammo_in_clip}")

            # If we've shot 20 times, trigger reload
            if self.ammo_in_clip == 0:
                print("Clip empty! Reloading...")
                self.reload()

        else:
            print("No ammo in clip! Reload first.")

    def reload(self):
        if self.reload_in_progress:
            print("Already reloading...")
            return
        
        if self.ammo_count == 0:
            print("No more ammo available to reload.")
            return
        
        # Start the reload process
        self.reload_in_progress = True
        print(f"Reloading... Please wait {self.player_config.reload_time} seconds.")
        
        # Simulate reload time (2 seconds)
        self.task_mgr.doMethodLater(self.player_config.reload_time, self.finish_reload, 'finish_reload')

    def setup_environment(self):
        # Create ground
        ground_shape = BulletBoxShape(Vec3(50, 50, 0.1))
        ground_node = BulletRigidBodyNode('Ground')
        ground_node.add_shape(ground_shape)
        ground_node.set_friction(0.5)
        ground_np = self.render.attach_new_node(ground_node)
        ground_np.set_pos(0, 0, -0.1)
        self.bullet_world.attach_rigid_body(ground_node)

        # Add a visual model for the ground
        ground_model = self.loader.load_model("models/box")
        ground_model.set_scale(100, 100, 0.2)
        ground_model.set_pos(0, 0, -0.1)
        ground_model.reparent_to(self.render)
        ground_model.set_color(0.5, 0.5, 0.5, 1)  # Gray color

        # Add lighting
        dlight = DirectionalLight('dlight')
        dlight.set_color((0.8, 0.8, 0.8, 1))
        dlnp = self.render.attach_new_node(dlight)
        dlnp.set_hpr(45, -45, 0)
        self.render.set_light(dlnp)

        alight = AmbientLight('alight')
        alight.set_color((0.2, 0.2, 0.2, 1))
        alnp = self.render.attach_new_node(alight)
        self.render.set_light(alnp)

    def mouse_look(self, task):
        if self.mouseWatcherNode.hasMouse():
            # Get mouse position and center position
            mouse = self.mouseWatcherNode.getMouse()
            
            # Update camera orientation
            self.camera.setH(self.camera.getH() - mouse.getX() * 20)
            self.camera.setP(max(min(self.camera.getP() + mouse.getY() * 20, 80), -80))
            
            # Center mouse
            self.win.movePointer(0, 
                               int(self.win.getXSize() / 2), 
                               int(self.win.getYSize() / 2))

        return Task.cont

    def update(self, task):
        dt = globalClock.get_dt()
        self.bullet_world.do_physics(dt)

        # Handle keyboard input for movement
        is_down = self.mouseWatcherNode.is_button_down
        
        if is_down("w"):
            self.move_forward()
        if is_down("s"):
            self.move_backward()
        if is_down("a"):
            self.move_left()
        if is_down("d"):
            self.move_right()
        if is_down("space"):
            self.jump()

        # Update camera position to follow player
        player_pos = self.player_np.get_pos()
        self.camera.set_pos(player_pos + Vec3(0, 0, 1))  # Offset camera above player

        return Task.cont

    def finish_reload(self, task):
        # Calculate how much ammo we can reload
        ammo_needed = self.player_config.ammo_per_reload - self.ammo_in_clip
        ammo_available = min(ammo_needed, self.ammo_count)
        
        self.ammo_in_clip += ammo_available
        self.ammo_count -= ammo_available
        self.reload_in_progress = False
        
        print(f"Reload complete! Ammo in clip: {self.ammo_in_clip}, Total ammo remaining: {self.ammo_count}")
        return Task.done

    def create_hit_effect(self, hit_point):
        # Create a visual effect at the hit point (e.g., a small explosion or a spark)
        # This is a placeholder, you can implement your own effect here
        print("Hit effect at:", hit_point)

if __name__ == "__main__":
    app = FPSApp()
    app.run()