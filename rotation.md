# Rotation Findings

## Summary

- The CSV rows are treated as absolute poses in robot-base space.
- Frame-0 CSV positions approximately match Datasmith rest translations after the script's mm->cm and XZY remap.
- Frame-0 CSV rotations do **not** match Datasmith rest rotations directly.
- The script uses frame 0 to derive a fixed rotation offset between the CSV orientation basis and the Datasmith rest orientation basis.

Relevant code:

- `main.py:83-84`
- `main.py:107`
- `main.py:320-359`

## How `main.py` uses the CSV

For each moving part, the script builds a world transform from:

- translation: taken directly from the CSV after unit and axis conversion
- rotation: taken from the CSV, but normalized relative to frame 0 before being applied to the Datasmith rest rotation

The key code is:

```python
current_rot = csv_rotation_to_scene_space(sample.angles_deg, order=order, intrinsic=intrinsic, np=np)
delta_rot = current_rot @ first_rot_by_mesh[part.mesh_name].T
rest_rot = rest_by_mesh[part.mesh_name][:3, :3]

matrix = compose_transform(
    delta_rot @ rest_rot,
    csv_position_to_scene_space(sample.pos_mm, np=np),
    np=np,
)
```

This means:

```text
final_rot = current_rot @ first_rot.T @ rest_rot
```

So the fixed offset between CSV frame 0 and Datasmith rest is:

```text
offset = first_rot.T @ rest_rot
final_rot = current_rot @ offset
```

At frame 0:

```text
final_rot = first_rot @ first_rot.T @ rest_rot = rest_rot
```

So the rendered frame-0 orientation is exactly the Datasmith rest rotation.

## Frame-0 translation vs Datasmith

The code comments already expect frame-0 translations to match Datasmith rest translations:

```python
# Link translations already match the frame-0 Datasmith positions in cm after x,z,y mapping.
```

Measured on the current dataset, the frame-0 translation differences are small:

- `Link_1-1_Mesh`: about `0.05 cm`
- `Link_2-2_Mesh`: about `0.017 cm`
- `Link_3-1_Mesh`: about `0.05 cm`
- `Link_4-1_Mesh`: about `0.05 cm`
- `Link_5-1_Mesh`: about `0.071 cm`
- `Flange-1_Mesh`: about `0.071 cm`

Conclusion: frame-0 positions are effectively the same as Datasmith rest positions for this repo.

## Frame-0 rotation vs Datasmith

Measured on the current dataset, the raw frame-0 CSV rotations for the moving links are about `90 deg` away from the Datasmith rest rotations.

Conclusion: frame-0 rotations are **not** the same as Datasmith rest rotations before normalization.

## `Link_1-1` worked example

CSV frame 0:

```text
TIME,link_1_ox,link_1_oy,link_1_oz,link_1_e1,link_1_e2,link_1_e3
0.00E+00,3.38E-14,3.49E+02,-1.07E-13,9.00E+01,9.00E+01,9.00E+01
```

Datasmith rest transform:

```xml
<Transform tx="0.0" ty="0.000002" tz="34.850002" sx="1.0" sy="1.0" sz="1.0"
           qx="0.5" qy="0.5" qz="0.5" qw="0.5" />
```

After conversion through `main.py`:

CSV frame-0 rotation matrix:

```text
[[ 0, 1, 0],
 [ 1, 0, 0],
 [ 0, 0,-1]]
```

Datasmith rest rotation matrix:

```text
[[0, 0, 1],
 [1, 0, 0],
 [0, 1, 0]]
```

Their relationship is:

```text
offset = first_rot.T @ rest_rot
       = [[1, 0, 0],
          [0, 0, 1],
          [0,-1, 0]]
       = Rx(-90 deg)
```

So for `Link_1-1`:

```text
rest_rot = first_rot @ Rx(-90 deg)
```

That means the Datasmith rest orientation is the CSV frame-0 orientation followed by a fixed `-90 deg` rotation around the X axis.

## Practical interpretation

- The CSV seems to provide absolute part poses in robot-base space.
- The CSV translations line up with Datasmith rest translations at frame 0.
- The CSV rotations use a different orientation basis or zero-reference than Datasmith.
- `main.py` compensates by treating frame 0 as the alignment frame and applying subsequent CSV motion on top of the Datasmith rest orientation.
