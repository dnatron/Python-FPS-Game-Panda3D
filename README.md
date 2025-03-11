# FPS Game with Panda3D and ODE Physics

A First Person Shooter sample game implemented using Panda3D and Open Dynamics Engine (ODE) simulation.

## Overview

This project demonstrates the integration of ODE physics with Panda3D to create a simple first-person shooter small sample code. The physics simulation provides realistic movement, collisions, and interactions between objects in the game world.

## Features

- First-person camera control with mouse look
- Realistic physics using ODE (Open Dynamics Engine)
- Physics-based movement with proper momentum and friction
- Shooting mechanics with raycast
- Ammo system with reloading
- Physics boxes that react realistically to collisions
- Optimized physics simulation with stability enhancements
- Custom damping for realistic motion

## Requirements

- Python 3.7+
- Panda3D 1.10.13 or higher (with ODE support)

## Installation

1. Install the required dependencies:
```bash
pip install -r requirements.txt
```

2. Ensure Panda3D is installed with ODE support:
```bash
pip install panda3d
```

## Running the Game

Run the game using:
```bash
python3 main.py
```

## Controls

- **W**: Move forward
- **S**: Move backward
- **A**: Move left
- **D**: Move right
- **Mouse**: Look around
- **Left Mouse Button**: Shoot
- **R**: Reload weapon
- **Space**: Jump

## Physics Implementation Details

The game uses ODE (Open Dynamics Engine) for physics simulation, which provides the following benefits:

1. **Realistic Collisions**: Objects collide and bounce with realistic physics properties
2. **Momentum-Based Movement**: Player and objects move with proper momentum and inertia
3. **Gravity Effects**: Objects fall naturally under the influence of gravity
4. **Stable Simulation**: Multiple substeps and optimized parameters ensure stable physics

Key physics components:
- `OdeWorld`: The main physics world that manages all physics objects
- `OdeSpace`: Handles collision detection between objects
- `OdeJointGroup`: Manages contact joints created during collisions
- `OdeBody`: Represents rigid bodies with mass and inertia
- `OdeBoxGeom`: Collision geometry for box-shaped objects

## Customizing Physics

You can adjust the physics behavior by modifying these parameters in the code:

- **Gravity**: Change the `physics_world_config.gravity` value
- **Surface Properties**: Modify the `setSurfaceEntry` parameters for different friction and bounce
- **Mass Values**: Adjust the mass of objects for different weight behaviors
- **Force Magnitudes**: Change the force values in movement methods for different control feels
- **Damping**: Modify the damping values in `apply_damping()` for different air resistance effects

## Troubleshooting

If you encounter issues:

1. Make sure Panda3D is installed with ODE support
2. Check that all required models are in the correct paths
3. Verify Python version compatibility (3.7+ recommended)
4. If physics seem unstable, try adjusting the physics parameters in the code

## Future Improvements

- Add more complex physics objects and interactions
- Implement advanced weapon systems
- Add enemy AI with physics-based movement
- Create more detailed environments with physics-based destruction
