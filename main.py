from dataclasses import dataclass
from panda3d.core import Vec3, CollisionNode, CollisionRay, DirectionalLight, AmbientLight, CollisionHandlerQueue, CollisionTraverser, WindowProperties, BitMask32, Vec4, ClockObject, Quat
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.ode import OdeWorld, OdeSimpleSpace, OdeJointGroup, OdeBody, OdeMass, OdeBoxGeom, OdePlaneGeom
from math import sin, cos, radians


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
            PhysicsBoxConfig(position=Vec3(7, 3, 4), size=Vec3(0.5, 0.5, 0.5), mass=1.0, friction=0.5),
            PhysicsBoxConfig(position=Vec3(3, 7, 6), size=Vec3(0.5, 0.5, 0.5), mass=1.0, friction=0.5),
            PhysicsBoxConfig(position=Vec3(2, 2, 8), size=Vec3(0.5, 0.5, 0.5), mass=1.0, friction=0.5),
        ]

        # Setup the physics world with ODE
        self.ode_world = OdeWorld()
        self.ode_world.setGravity(self.physics_world_config.gravity.x, 
                                  self.physics_world_config.gravity.y, 
                                  self.physics_world_config.gravity.z)
        
        # Initialize surface table for collision properties - improved for realism
        self.ode_world.initSurfaceTable(1)
        # Parameters: surface1, surface2, bounce, bounce_vel, friction, softness, slip, dampen, erp
        # Reduced bounce, increased friction, adjusted softness for more realistic collisions
        self.ode_world.setSurfaceEntry(0, 0, 0.2, 0.1, 0.9, 0.005, 0.001, 0.5, 0.2)
        
        # Set CFM (Constraint Force Mixing) for softer constraints
        self.ode_world.setCfm(0.001)
        
        # Set ERP (Error Reduction Parameter) for better stability
        self.ode_world.setErp(0.8)
        
        # Increase iterations for more accurate simulation
        self.ode_world.setQuickStepNumIterations(20)
        
        # Create a space and add a contactgroup for collision handling
        self.ode_space = OdeSimpleSpace()
        self.ode_space.setAutoCollideWorld(self.ode_world)
        self.contact_group = OdeJointGroup()
        self.ode_space.setAutoCollideJointGroup(self.contact_group)
        
        self.setup_environment()

        # Setup the player and its physics
        self.create_player()

        # Setup the physics boxes
        self.box_objects = []
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
        self.player.setColor(0.2, 0.6, 1.0, 1.0)  # Blue color for player

        # Physics setup for the player (ODE)
        self.player_body = OdeBody(self.ode_world)
        mass = OdeMass()
        # Create a box mass with the given parameters (density, lx, ly, lz)
        # Increased mass for better stability
        mass.setBox(self.player_config.mass * 5, 1.0, 1.0, 1.0)
        self.player_body.setMass(mass)
        self.player_body.setPosition(self.player_config.position)
        
        # Create collision geometry for the player
        self.player_geom = OdeBoxGeom(self.ode_space, 1.0, 1.0, 1.0)
        self.player_geom.setCollideBits(BitMask32(0x00000001))
        self.player_geom.setCategoryBits(BitMask32(0x00000002))
        self.player_geom.setBody(self.player_body)

    def create_physics_box(self, box_config: PhysicsBoxConfig):
        # Create a visual model for the box
        box = self.loader.load_model("models/box")
        box.set_scale(box_config.size)
        box.reparent_to(self.render)
        box.set_pos(box_config.position)
        box.setColor(0.8, 0.3, 0.2, 1.0)  # Red color for boxes

        # Create ODE body for the box
        box_body = OdeBody(self.ode_world)
        mass = OdeMass()
        # Create a box mass with the given parameters (density, lx, ly, lz)
        # Adjusted density for more realistic mass
        density = box_config.mass * 2.0
        mass.setBox(density, 
                   box_config.size.x * 2, 
                   box_config.size.y * 2, 
                   box_config.size.z * 2)
        box_body.setMass(mass)
        box_body.setPosition(box_config.position)
        
        # Create collision geometry for the box
        box_geom = OdeBoxGeom(self.ode_space, 
                             box_config.size.x, 
                             box_config.size.y, 
                             box_config.size.z)
        box_geom.setCollideBits(BitMask32(0x00000003))  # Collide with everything
        box_geom.setCategoryBits(BitMask32(0x00000001))
        box_geom.setBody(box_body)
        
        # Store the box and its body for updating
        self.box_objects.append((box, box_body))

    def move_forward(self):
        # Get the direction vector from the camera
        direction = Vec3(self.camera.getMat().getRow3(1))
        direction.z = 0  # Keep movement on the horizontal plane
        direction.normalize()
        
        # Apply force to move in that direction - adjusted for better control
        self.player_body.addForce(direction * self.speed * 150)

    def move_backward(self):
        # Get the direction vector from the camera
        direction = Vec3(self.camera.getMat().getRow3(1))
        direction.z = 0  # Keep movement on the horizontal plane
        direction.normalize()
        
        # Apply force to move in the opposite direction - adjusted for better control
        self.player_body.addForce(-direction * self.speed * 150)

    def move_left(self):
        # Get the right vector from the camera and negate it for left
        direction = Vec3(self.camera.getMat().getRow3(0))
        direction.z = 0  # Keep movement on the horizontal plane
        direction.normalize()
        
        # Apply force to move left - adjusted for better control
        self.player_body.addForce(-direction * self.speed * 150)

    def move_right(self):
        # Get the right vector from the camera
        direction = Vec3(self.camera.getMat().getRow3(0))
        direction.z = 0  # Keep movement on the horizontal plane
        direction.normalize()
        
        # Apply force to move right - adjusted for better control
        self.player_body.addForce(direction * self.speed * 150)

    def jump(self):
        # Apply upward force to simulate a jump
        current_vel = self.player_body.getLinearVel()
        current_pos = self.player_body.getPosition()
        
        # Only jump if we're close to the ground (checking position and velocity)
        if current_pos.z < 1.5 and abs(current_vel.z) < 0.5:
            self.player_body.addForce(Vec3(0, 0, 8000))

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
        # Create ground plane for ODE
        ground_geom = OdePlaneGeom(self.ode_space, Vec4(0, 0, 1, 0))
        ground_geom.setCollideBits(BitMask32(0x00000003))  # Collide with everything
        ground_geom.setCategoryBits(BitMask32(0x00000002))
        
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
        dt = ClockObject.getGlobalClock().getDt()
        
        # Limit dt to avoid instability in physics
        dt = min(dt, 0.05)
        
        # Perform collision detection
        self.ode_space.autoCollide()
        
        # Step the simulation with smaller substeps for stability
        for i in range(4):  # 4 substeps for better stability
            self.ode_world.quickStep(dt / 4)
            
        # Apply a maximum velocity limit to prevent objects from moving too fast
        self.limit_velocities()
        
        # Apply manual damping to all bodies for more realistic motion
        self.apply_damping()
        
        # Update visual positions of all objects
        # Update player position
        player_pos = self.player_body.getPosition()
        self.player.setPos(player_pos)
        
        # Update box positions
        for box, body in self.box_objects:
            box.setPos(body.getPosition())
            # Get the rotation quaternion from ODE
            quat = body.getQuaternion()
            # Set the rotation on the visual model - create a Panda3D quaternion
            box.setQuat(Quat(quat[0], quat[1], quat[2], quat[3]))
        
        # Clear contact joints
        self.contact_group.empty()
        
        # Update camera position to follow player
        self.camera.set_pos(player_pos + Vec3(0, 0, 1))  # Offset camera above player

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

        return Task.cont
        
    def limit_velocities(self):
        # Limit velocities to prevent objects from moving too fast
        max_linear_speed = 15.0
        max_angular_speed = 3.0
        
        # Limit player velocity
        linear_vel = self.player_body.getLinearVel()
        speed = linear_vel.length()
        if speed > max_linear_speed:
            self.player_body.setLinearVel(linear_vel * (max_linear_speed / speed))
            
        angular_vel = self.player_body.getAngularVel()
        angular_speed = angular_vel.length()
        if angular_speed > max_angular_speed:
            self.player_body.setAngularVel(angular_vel * (max_angular_speed / angular_speed))
            
        # Limit box velocities
        for _, body in self.box_objects:
            linear_vel = body.getLinearVel()
            speed = linear_vel.length()
            if speed > max_linear_speed:
                body.setLinearVel(linear_vel * (max_linear_speed / speed))
                
            angular_vel = body.getAngularVel()
            angular_speed = angular_vel.length()
            if angular_speed > max_angular_speed:
                body.setAngularVel(angular_vel * (max_angular_speed / angular_speed))
        
    def apply_damping(self):
        # Apply manual damping to all bodies
        # This simulates air resistance and friction
        linear_damping = 0.05
        angular_damping = 0.2
        
        # Apply to player
        linear_vel = self.player_body.getLinearVel()
        self.player_body.setLinearVel(linear_vel * (1.0 - linear_damping))
        
        angular_vel = self.player_body.getAngularVel()
        self.player_body.setAngularVel(angular_vel * (1.0 - angular_damping))
        
        # Apply to boxes
        for _, body in self.box_objects:
            linear_vel = body.getLinearVel()
            body.setLinearVel(linear_vel * (1.0 - linear_damping))
            
            angular_vel = body.getAngularVel()
            body.setAngularVel(angular_vel * (1.0 - angular_damping))
        
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