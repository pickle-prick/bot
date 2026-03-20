# Pose Findings For `prob.py`

This note records the transform findings for the current `prob.py` parser.

## Position

The CSV position fields are not already in scene-space XYZ.

The correct frame-0 translation pattern is:

```text
(x_scene, y_scene, z_scene) = (x_csv, z_csv, y_csv) / 10
```

Reason:

- CSV positions are in millimeters.
- The OBJ / Datasmith scene is effectively in centimeters.
- The CSV Y and Z axes are swapped relative to the scene.

Example:

```text
link_1 frame 0 CSV: (0, 349, 0) mm
scene translation : (0, 0, 34.9) cm
```

That matches the authored link transform closely.

## Rotation

The raw CSV Euler rotation is not yet in the same coordinate frame as the scene.

If the CSV rotation is:

```text
R_csv = Rz(e1) Rx(e2) Rz(e3)
```

and the axis-swap matrix is:

```text
S = [1 0 0
     0 0 1
     0 1 0]
```

then the same physical rotation written in scene coordinates is:

```text
R_scene = S * R_csv * S^T
```

For this specific swap matrix, `S^T = S`, but conceptually it should still be written as `S^T` because this is a basis change.

## Why `S^T` Is Required

Vector coordinates change as:

```text
v_scene = S * v_csv
```

A rotation acts on vectors, so converting the rotation between coordinate systems requires converting both the input basis and the output basis:

```text
v'_csv   = R_csv * v_csv
v'_scene = S * v'_csv
         = S * R_csv * S^T * v_scene
```

So:

```text
R_scene = S * R_csv * S^T
```

Using only:

```text
S * R_csv
```

would mix coordinate systems. It would not be a proper scene-space rotation matrix.

## Fixed Mesh Offset

Even after converting the CSV rotation into scene coordinates, the mesh local frame is still offset from the CSV frame at frame 0.

The remaining constant correction is:

```text
Rx(-90)
```

So the full rotation is:

```text
R_final = S * R_csv * S^T * Rx(-90)
```

Interpretation:

- `S * R_csv * S^T` converts the CSV orientation into scene axes.
- `Rx(-90)` is a fixed local mesh correction.

Because `Rx(-90)` is on the right, it is applied in the mesh's local frame, which is what the frame-0 data indicates.

## Why `e1 + 90` Is Not Equivalent

Changing the parsed angles to:

```text
Rz(e1 + 90) Rx(e2) Rz(e3)
```

is not the same as:

```text
S * R_csv * S^T * Rx(-90)
```

Reason:

- adding `90` to `e1` changes the first Euler step inside the ordered decomposition
- the true issue is a basis conversion plus a separate fixed local correction
- Euler rotations do not generally commute, so these operations cannot be collapsed into `e1 += 90`

## Practical Summary

- Position fix: swap Y/Z and divide by 10.
- Rotation fix: convert the CSV rotation with `S * R_csv * S^T`.
- Then apply a fixed local `Rx(-90)` mesh correction.
- Do not treat the rotation issue as "just add 90 degrees to `e1`".

## Separate Note About `Tool-1`

`Tool-1` has no CSV track. If it is left at identity, its pose will not match the authored assembly rest transform.
