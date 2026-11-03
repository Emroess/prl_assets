# prl_assets

Reusable manipulation objects (cans, bins, tableware, containers) for MuJoCo simulation.

![Assets Preview](docs/assets_preview.png)

This package provides simulation models and metadata for common manipulation objects like cans, bottles, tableware, and containers. Objects follow the `asset_manager` metadata format for compatibility with perception and planning pipelines.

## Installation

```bash
uv add prl_assets
```

For development:
```bash
git clone https://github.com/personalrobotics/prl_assets.git
cd prl_assets
uv sync --dev
```

## Quick Start

Use with `asset_manager` to load objects:

```python
from asset_manager import AssetManager
from prl_assets import OBJECTS_DIR

# Initialize asset manager with prl_assets objects
assets = AssetManager(OBJECTS_DIR)

# List available objects
print(assets.list())
# ['can', 'cracker_box', 'fuze_bottle', 'gelatin_box', 'herring_tin',
#  'lunchbox', 'notebook', 'plastic_bowl', 'plastic_glass', 'plastic_plate',
#  'pocky_box', 'pop_tarts_case', 'spam_can', 'recycle_bin', 'sugar_box',
#  'wicker_tray', 'yellow_tote']

# Get path to simulation model
can_path = assets.get_path("can", "mujoco")

# Load in MuJoCo
import mujoco

model = mujoco.MjModel.from_xml_path(can_path)

# Get object metadata
meta = assets.get("can")
print(meta["dimensions"])  # [0.066, 0.066, 0.123]
print(meta["mass"])  # 0.05 kg

# Find objects by category
containers = assets.by_category("container")
```

## YCB-Video Collection

`objects/ycbv/` holds objects from the YCB-Video (BOP `ycbv`) set, so that a pose
estimator trained on YCB-V and the simulator refer to the same object, geometry and
frame. Load the collection alone, or the whole catalog, which includes it:

```python
from asset_manager import AssetManager
from prl_assets import OBJECTS_DIR, YCBV_OBJECTS_DIR

ycbv = AssetManager(YCBV_OBJECTS_DIR)  # only the YCB-V objects
catalog = AssetManager(OBJECTS_DIR)  # everything, YCB-V included

catalog.resolve_alias("003_cracker_box")  # 'cracker_box'
catalog.get("cracker_box")["dataset"]["ycbv"]["bop_id"]  # 2
```

Each YCB-V object records its BOP id, its original YCB name (also a perception alias),
the rigid transform `T_asset_bop` from the BOP model frame (converted to metres) to the
asset frame, the BOP symmetries, and its provenance. To place a detected object:
`T_camera_asset = T_camera_bop @ inverse(T_asset_bop)`, with the detector's translation
in metres. [docs/schema.md](docs/schema.md) defines these fields and frames;
[docs/ycbv_audit.md](docs/ycbv_audit.md) records how the existing objects were matched
to the BOP models. The scans are CC BY 4.0, not MIT; see
[objects/ycbv/NOTICE.md](src/prl_assets/objects/ycbv/NOTICE.md).

The collection currently holds `cracker_box`, `sugar_box`, `gelatin_box` and `spam_can`;
the remaining YCB-V objects are tracked in issue #28.

### Migrating from flat paths

`cracker_box`, `sugar_box`, `gelatin_box` and `spam_can` moved from `objects/<name>/`
to `objects/ycbv/<name>/`. Their names, aliases, models and physical properties did not
change, so code that looks them up through `AssetManager` is unaffected. Code that
builds paths such as `OBJECTS_DIR / "cracker_box"` must use
`AssetManager(OBJECTS_DIR).get_path("cracker_box", "mujoco")` instead.

## Available Objects

| Object | Description | Collision |
|--------|-------------|-----------|
| `can` | Aluminum soda/beer can | cylinder |
| `cracker_box` | Cheez-It crackers box (YCB-V 2) | box |
| `fuze_bottle` | Beverage bottle with texture | cylinder |
| `gelatin_box` | Jell-O gelatin box (YCB-V 8) | box |
| `herring_tin` | MW Polar herring tin | box + 2 cylinders |
| `lunchbox` | Wildkin insulated lunch box | box |
| `notebook` | Amazon Basics hardcover notebook | box |
| `plastic_bowl` | Tableware bowl (nestable) | mesh |
| `plastic_glass` | Drinking glass | cylinder |
| `plastic_plate` | Dinner plate (stackable) | mesh |
| `pocky_box` | Pocky biscuit sticks box | box |
| `pop_tarts_case` | Pop-Tarts food box | box |
| `spam_can` | SPAM canned meat (YCB-V 9, potted meat can) | box |
| `recycle_bin` | Open-top recycling bin (hollow) | 5 boxes |
| `sugar_box` | Domino sugar box (YCB-V 3) | box |
| `wicker_tray` | Serving tray (hollow) | 5 boxes |
| `yellow_tote` | Storage tote (hollow) | 5 boxes |

## Object Structure

Each object follows a standardized directory structure. Objects may be grouped in a
collection directory, such as `ycbv/`, which has no `meta.yaml` of its own:

```
objects/
├── object_name/
│   ├── meta.yaml          # Object metadata
│   ├── object_name.xml    # MuJoCo model
│   └── *_visual.obj       # Visual mesh (optional)
└── ycbv/
    └── cracker_box/       # same structure, plus YCB-V metadata
```

### Metadata Format

The `meta.yaml` file contains all object properties:

```yaml
name: can
description: Standard aluminum soda/beer can
category: [container, recyclable, graspable]

# Physical properties
mass: 0.05  # kg
dimensions: [0.066, 0.066, 0.123]  # meters [X, Y, Z] in resting pose; Z = height
material: aluminum

# Geometric properties for grasp planning
geometric_properties:
  type: cylinder
  radius: 0.033
  height: 0.123

# Simulator-specific configuration
mujoco:
  xml_path: can.xml
  scale: 1.0
  friction: [0.6, 0.005, 0.0001]

# Perception aliases for detection systems
perception:
  aliases: ["can", "soda can", "beer can"]

# Manipulation policy hints
policy:
  grasping:
    affordances: [lift, pour]
    preferred_grasp_type: side_grasp
    difficulty: easy
```

## Adding New Objects

1. Create a new directory under `src/prl_assets/objects/`
2. Add a `meta.yaml` with required fields (see format below)
3. Add MuJoCo XML model following the conventions below
4. Add visual mesh (`.obj` file) if needed
5. Verify with scripts:
   ```bash
   # Check coordinate frame and collision alignment
   uv run python scripts/inspect_object.py your_object

   # Validate against standards
   uv run python scripts/validate_objects.py your_object
   ```
6. Update the objects table in this README

### MuJoCo Model Requirements

Objects used with `mj_environment` must follow these conventions:

```xml
<mujoco model="object_name">
  <worldbody>
    <body name="object_name" pos="0 0 {half_height}">
      <freejoint name="object_name_joint"/>  <!-- Required for positioning -->
      <!-- geoms, sites, etc. -->
    </body>
  </worldbody>
</mujoco>
```

- **freejoint**: Required for `registry.activate()` to set object position/orientation
- **body name**: Should match the object type (becomes `{type}_{index}` when loaded)

### Coordinate Frame Standard

All objects must follow this coordinate frame convention:

1. **Origin at bottom center**: The body origin is at the center of the object's base (bottom surface) in its canonical resting pose
2. **Z-up orientation**: The object's primary axis is aligned with +Z
3. **Rests on z=0**: When placed at position (x, y, 0), the object sits on a z=0 surface
4. **Canonical resting pose**: The stable pose the object would naturally rest in (e.g., can upright, box on largest face)
5. **Dimensions in resting frame**: The `dimensions` field in meta.yaml is [X, Y, Z] in the canonical resting pose, where **Z is always the height when resting**. For a flat object (e.g., herring tin) that rests on its largest face, Z is the thin dimension.

**Implementation**: The body `pos` attribute should offset the origin by half the object's height (dimensions[2]/2):
```xml
<!-- For a cylinder of height 0.123m -->
<body name="can" pos="0 0 0.0615">
```

**Collision and visual alignment**: Collision geoms must be aligned with visual meshes. Both should be centered at the body origin with matching dimensions.

**YCB-V objects**: the body `pos` is only the placement that rests the object on z=0
when its file is loaded alone; it does not change the object's frame. A YCB-V object's
geometry is expressed about the BOP model origin, near its geometric centre, and the
relation to the BOP frame is `T_asset_bop`. See [docs/schema.md](docs/schema.md#frames-framest_asset_bop).

**Verification**: Run visualization scripts to verify object geometry:

```bash
# Inspect a single object with collision overlay
uv run python scripts/inspect_object.py can

# Visualize all objects in a grid
uv run python scripts/viz_all_objects.py

# Physics demo with stacking
uv run python scripts/viz_stacking_demo.py

# Validate all objects against standards
uv run python scripts/validate_objects.py
```

## Scripts

| Script | Description |
|--------|-------------|
| `inspect_object.py` | Visualize single object with collision overlay and coordinate axes |
| `viz_all_objects.py` | Display all 17 objects in a grid with collision geometry |
| `viz_stacking_demo.py` | Physics demo: stacked dishes on tray, objects in tote |
| `validate_objects.py` | Check objects against coordinate frame and collision standards |
| `recenter_mesh.py` | Utility to re-center mesh origin to centroid |
| `audit_ycbv.py` | Check YCB-V objects' declared frames against the pinned BOP models by shape and texture; writes `artifacts/ycbv_audit.json`, and with `--record` `docs/ycbv_audit.json` |

## License

The code is MIT licensed; see LICENSE. Object data keeps its source's terms: the YCB-V
scans are CC BY 4.0, as stated in
[objects/ycbv/NOTICE.md](src/prl_assets/objects/ycbv/NOTICE.md).
