# Scene Creation Tutorial


## Quick Start (15-minute photorealistic warehouse)

This tutorial creates your first photorealistic warehouse scene with H1 humanoid robot, ready for training computer vision models.

### Step 1: Launch Isaac Sim with Template (30 seconds)

```bash
# Start Isaac Sim in simulation-friendly mode
./isaac-sim.sh \
  --load-scene="/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd" \
  --no-window \
  --simulate-continuous \
  --headless > launch.log 2>&1

# Verify successful launch
grep "Warehouse scene loaded" launch.log
```

### Step 2: Add Humanoid Robot (2 minutes)

Using the Isaac Sim Python API:

```python title="warehouse_scene_setup.py - minimal approach"
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.core.utils.numpy as np_utils
from pxr import UsdGeom, Gf

def create_warehouse_with_humanoid():
    # Get the stage
    stage = omni.usd.get_context().get_stage()

    # Add H1 humanoid at warehouse origin
    humanoid_path = "/World/humanoid_h1"
    prim_utils.create_prim(
        prim_path=humanoid_path,
        usd_path="/Isaac/Robots/Unitree/H1/h1.usd",
        translation=(0, 0, 0)
    )

    # Position humanoid naturally near work area
    prim_utils.set_prim_property(
        prim_path + "/skeleton_root"
        property_name="xformOp:translate",
        value=Gf.Vec3f(2.0, 0.0, 0.0)  # 2 meters ahead
    )

    # Set canonical humanoid height
    prim_utils.set_prim_property(
        humanoid_path,
        property_name="xformOp:scale",
        value=Gf.Vec3f(1.0, 1.0, 1.0)
    )

    return humanoid_path

create_warehouse_with_humanoid()  # Execute
```

### Step 3: Configure Lighting for AI Training (3 minutes)

```python title="scene_lighting_configuration.py"
import omni.isaac.core.utils.prims as prim_utils
from pxr import Usd
from pxr import UsdLux

def setup_warehouse_lighting():
    # Remove default omniverse lights
    stage = omni.usd.get_context().get_stage()

    # Add area lights for realistic warehouse illumination
    light_types = [
        ("/World/Light1", "_sky_light"),  # Soft ambient lighting
        ("/World/Light2", "_overhead_light"),
        ("/World/Light3", "_ambient_bounce")
    ]

    for light_name, light_type in light_types:
        usd_light = UsdLux.SphereLight.Define(stage, light_name)
        usd_light.CreateRadiusAttr(0.5)

        if light_type == "_sky_light":
            usd_light.CreateColorAttr((1.0, 1.0, 1.05))  # Cool white
            usd_light.CreateIntensityAttr(5000)  # 500W area equivalent

        elif light_type == "_overhead_light":
            usd_light.CreateColorAttr((1.02, 1.0, 0.98))  # Slight warm    BSD window
            usd_light.CreateIntensityAttr(8000)  # Main workplace illumination

        elif light_type == "_ambient_bounce":
            usd_light.CreateColorAttr((0.98, 0.98, 1.02))  # Blue bounce from walls
            usd_light.CreateIntensityAttr(3000)
```

### Step 4: Add Functional Objects (3 minutes)

```python title="add_working_objects.py"
def populate_warehouse_objects():
    # Realistic working environment objects
    objects_config = [
        {
            "path": "/World/object_weighing_pallet",
            "type": "pallet",
            "position": (3.5, -1.0, 0.15),  # naturally in humanoid workspace
            "variants": [
                "/Isaac/Assets/SimReady/Standard/Engineering/cajaPileBox.usd",
                "/Isaac/Assets/SimReady/Standard/Engineering/greenCrate.usd"
            ]
        },
        {
            "path": "/World/work_sampling_tools",
            "type": "tools",
            "position": (2.5, 2.0, 0.6),
            "variants": "/Isaac/Assets/SimReady/Standard/Engineering/toolsBox.usd"
        }
    ]

    for obj_config in objects_config:
        # Random variant selection
        import random
        selected_variant = random.choice(obj_config["variants"])

        # Create object instance
        prim_utils.create_prim(
            obj_config["path"],
            usd_path=selected_variant,
            translation=obj_config["position"],
            scale=(0.5, 0.5, 0.5)  # Adjust size for H1 humanoid scale
        )
```

### Step 5: Position Cameras for Training (4 minutes)

```python title="instrument_camera}.py" This creates camera setup for CV training as per chapter 3 requirements

import omni.isaac.core.utils.prims as prim_utils
import carb
import numpy as np

def configure_training_cameras():
    """
    Configure stereo camera rig for humanoid robot training
    Matches H1 humanoid head position and orientation
    """

    # Humanoid head position (maintains natural perspective)
    base_height = 1.6  # H1 natural height

    def create_camera_config(name, relative_pos):
        """Helper to create camera rig configurations"""

        # Stereo baseline - human-like spacing
        baseline = 0.12  # 12cm - natural human eye separation

        # Primary viewpoint from humanoid head perspectives
        if name == "left_head_camera":
            pos = (0.0, baseline/2, base_height)
        elif name == "right_head_camera":
            pos = (0.0, -baseline/2, base_height)
        else:  # Overview camera
            pos = (-2.0, 0.5, base_height+0.3)

        # Apply relative offset
        offset = np.array(relative_pos)

        return tuple(np.array(pos) + offset)

    # Camera rig configurations for different training viewpoints
    camera_configs = {
        "object_detection": [
            ("left_head_camera", (0.15, 0.0, 0.12)),
            ("right_head_camera", (0.15, 0.0, 0.12)),
            ("overview_camera", (0.0, 0.0, 0.0))  # Static overview
        ],
        "navigation": [
            ("nav_eye_left", (0.0, 0.06, base_height-0.02)),
            ("nav_eye_right", (0.0, -0.06, base_height-0.02)),
            ("nav_chest", (-0.1, 0.0, base_height-0.3))  # Chest-level obstacle view
        ]
    }

    # Process camera configurations
    cameras = []
    for config_name, positions in camera_configs.items():
        for cam_name, rel_pos in positions:
            camera_path    US1 summary with quiz and outcomes"}].file_path is not a valid parameter of Write:0. "file_path" is None