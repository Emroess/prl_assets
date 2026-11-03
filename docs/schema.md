# Object metadata schema

This document specifies the sections of `meta.yaml` that prl_assets owns beyond those
asset_manager interprets. asset_manager's
[schema](https://github.com/personalrobotics/asset_manager/blob/main/docs/schema.md)
owns discovery, `name`, `category`, the `mujoco` path, and `perception.aliases`; it
stores every other section unchecked, so prl_assets' tests validate the sections below.

## Collections

An object is a directory holding a `meta.yaml`. Objects may be grouped in collection
directories, which hold no `meta.yaml` of their own. `prl_assets.OBJECTS_DIR` is the
whole catalog, collections included; `prl_assets.YCBV_OBJECTS_DIR` is the YCB-Video
collection alone. Canonical names are unique across the whole catalog, and a
collection does not prefix them.

## YCB-Video identity: `dataset.ycbv`

Every object in `objects/ycbv/` declares which YCB-Video object it is:

```yaml
dataset:
  ycbv:
    bop_id: 2                 # BOP ycbv object id, 1-21
    ycb_name: 003_cracker_box # the original YCB model name
```

`bop_id` and `ycb_name` follow the BOP release's numbering, which is contiguous and
differs from the numeric prefixes of the YCB names. Each id names exactly one object in
the catalog. A detector's class index is not an asset identity; map it to `bop_id`
at the perception boundary. Every YCB-V object also lists its `ycb_name` among its
`perception.aliases`, so `resolve_alias("003_cracker_box")` returns its canonical name.

## Frames: `frames.T_asset_bop`

Three frames are involved:

- **BOP model frame.** The frame of the reference model `models/obj_<bop_id>.ply` in the
  BOP release, in millimetres. Pose estimators trained on YCB-V report
  `T_camera_bop` in this frame.
- **Asset frame.** The frame in which the object's visual mesh and collision geometry
  are expressed, in metres. It is the MuJoCo body frame: the model's geoms carry no
  offset from the body.
- **Resting placement.** The body's `pos` (and `quat`, if any) in the model file only
  places the object in the world when the file is loaded on its own, resting on
  `z = 0`. It is not part of the asset frame, and a scene that sets the free joint's
  pose replaces it.

`T_asset_bop` is a 4×4 homogeneous rigid transform, stored as a list of four rows,
mapping BOP-frame points, **after conversion from millimetres to metres**, into the
asset frame:

```text
p_asset         = T_asset_bop @ (p_bop_mm / 1000)
T_camera_asset  = T_camera_bop_m @ inverse(T_asset_bop)
```

where `T_camera_bop_m` is the detector's pose with its translation converted to metres.
Scale never enters `T_asset_bop`; its rotation block is orthonormal with determinant 1.

The four objects that predate the collection keep their frames, so existing scenes
behave as before:

| Object | `T_asset_bop` rotation | Why |
| --- | --- | --- |
| `cracker_box` | identity | |
| `spam_can` | identity | |
| `sugar_box` | 90° about y | laid on its largest face |
| `gelatin_box` | −13.64° about z | edges aligned with the asset axes |

Translations are zero: the asset and BOP origins coincide, at the BOP model origin near
the geometric centre. New objects in the collection use the identity.

`scripts/audit_ycbv.py` checks each declared transform against the BOP model, by shape
and by texture, and records the result in `docs/ycbv_audit.json`; see
[the audit](ycbv_audit.md).

## Symmetries: `symmetries`

```yaml
symmetries:
  frame: bop      # the BOP model frame, millimetres
  discrete: []    # 4x4 transforms, as 16 row-major numbers each
  continuous: []  # {axis: [x, y, z], offset: [x, y, z]}
```

The entries are copied unchanged from the BOP release's `models_info.json`
(`symmetries_discrete`, `symmetries_continuous`) and stay in its frame and units. Empty
lists mean the release declares no symmetry. A consumer working in the asset frame
converts a discrete symmetry `S` (translation in millimetres) as
`T_asset_bop @ S_m @ inverse(T_asset_bop)`, where `S_m` has its translation divided by
1000.

## Provenance: `provenance`

```yaml
provenance:
  geometry: where the shipped mesh came from, and how it was produced
  reference_model: the BOP file this object is checked against
  license: SPDX identifier of the mesh and texture license
  attribution: where the full notice lives
```

The full notices for the YCB-V collection are in `objects/ycbv/NOTICE.md`, which ships
with the package. The package's MIT license does not relicense object data.
