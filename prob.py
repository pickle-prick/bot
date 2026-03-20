import sys
import csv
import math
import time
import trimesh
import pyrender
import numpy as np
from dataclasses import dataclass

type Color = tuple[int,int,int]
setattr(np, "infty", np.inf)

# FIXME: maybe we don't need this, we could just compute a offset matrix from known values
OBJ_TO_SCENE_BASIS = (
  (1.0, 0.0, 0.0, 0.0),
  (0.0, 0.0, 1.0, 0.0),
  (0.0, 1.0, 0.0, 0.0),
  (0.0, 0.0, 0.0, 1.0),
)

@dataclass(frozen=True)
class PartSpec:
  name: str
  obj_path: str
  csv_path: str|None
  color_u32: Color

@dataclass(frozen=True)
class PoseSample:
  time_s: float
  pos_mm: tuple[float,float,float]
  angles_deg: tuple[float,float,float]

PART_SPECS: tuple[PartSpec, ...] = (
  PartSpec("Base-1",   "objs/Base-1_Mesh.OBJ",   None,                   (16, 16, 16)),
  PartSpec("Link_1-1", "objs/Link_1-1_Mesh.OBJ", "Robot/link_1_res.csv", (210, 60, 60)),
  PartSpec("Link_2-2", "objs/Link_2-2_Mesh.OBJ", "Robot/link_2_res.csv", (235, 170, 55)),
  PartSpec("Link_3-1", "objs/Link_3-1_Mesh.OBJ", "Robot/link_3_res.csv", (65, 135, 220)),
  PartSpec("Link_4-1", "objs/Link_4-1_Mesh.OBJ", "Robot/link_4_res.csv", (90, 185, 120)),
  PartSpec("Link_5-1", "objs/Link_5-1_Mesh.OBJ", "Robot/link_5_res.csv", (125, 90, 205)),
  PartSpec("Flange-1", "objs/Flange-1_Mesh.OBJ", "Robot/flange_res.csv", (60, 170, 170)),
  PartSpec("Tool-1",   "objs/Tool-1_Mesh.OBJ",    None,                  (80, 80, 80)),
)

@dataclass(frozen=True)
class Part:
  spec: PartSpec
  samples: list[PoseSample]
  sample_duration: float
  mesh: trimesh.Mesh
  bounds: tuple[np.ndarray, np.ndarray]

def make_look_at(eye, target, up_hint):
  forward = target - eye
  forward /= np.linalg.norm(forward)
  right = np.cross(forward, up_hint)
  if np.linalg.norm(right) < 1e-6:
    right = np.array([1.0, 0.0, 0.0], dtype=float)
  else:
    right /= np.linalg.norm(right)
  up = np.cross(right, forward)
  up /= np.linalg.norm(up)

  pose = np.eye(4, dtype=float)
  pose[:3, 0] = right
  pose[:3, 1] = up
  pose[:3, 2] = -forward
  pose[:3, 3] = eye
  return pose

def samples_from_csv(csv_path: str, prefix: str) -> list[PoseSample]:
  rows: list[PoseSample] = []
  with open(csv_path, "r", encoding="utf-8", newline="") as handle:
    reader = csv.DictReader(handle)
    for row in reader:
      rows.append(
        PoseSample(
          time_s=float(row["TIME"]),
          pos_mm=(
            0.1*float(row[f"{prefix}_ox"]),
            0.1*float(row[f"{prefix}_oy"]),
            0.1*float(row[f"{prefix}_oz"]),
          ),
          angles_deg=(
            float(row[f"{prefix}_e1"]),
            float(row[f"{prefix}_e2"]),
            float(row[f"{prefix}_e3"])
          )
        )
      )
  return rows

def rotation_axis_matrix(axis_digit: str, angle_deg: float):
  angle_rad = math.radians(angle_deg)
  c = math.cos(angle_rad)
  s = math.sin(angle_rad)

  if axis_digit == "1":
    return np.array(
      [
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
        ],
      dtype=float,
    )
  if axis_digit == "2":
    return np.array(
      [
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c],
      ],
      dtype=float,
    )
  if axis_digit == "3":
    return np.array(
      [
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
      ],
      dtype=float,
    )
  raise ValueError(f"Unsupported Euler axis {axis_digit!r}")

def mat4_from_translate(translate: tuple[float,float,float]) -> np.ndarray:
  mat = np.eye(4, dtype=float)
  mat[:3, 3] = np.array(translate, dtype=float)
  return mat

def mat4_from_euler(order: str, angles_deg: tuple[float,float,float], intrinsic: bool):
  a1, a2, a3 = angles_deg
  pieces = [
    rotation_axis_matrix(order[0], a1),
    rotation_axis_matrix(order[1], a2),
    rotation_axis_matrix(order[2], a3),
  ]

  mat3 = np.eye(3, dtype=float)
  if intrinsic:
    for piece in pieces:
      mat3 = mat3 @ piece
  else:
    for piece in pieces:
      mat3 = piece @ mat3

  ret = np.eye(4, dtype=float)
  ret[:3, :3] = mat3
  return ret

def part_from_spec(spec: PartSpec):
  tri = trimesh.load(spec.obj_path, force="mesh", process=False)
  # if isinstance(tri, trimesh.Scene):
  #   tri = trimesh.util.concatenate(tuple(tri.geometry.values()))
  basis = np.array(OBJ_TO_SCENE_BASIS, dtype=float)
  tri.apply_transform(basis)
  color = tuple(channel / 255.0 for channel in spec.color_u32) + (1.0,)
  material = pyrender.MetallicRoughnessMaterial(
    baseColorFactor=color,
    metallicFactor=0.0,
    roughnessFactor=0.9,
    doubleSided=False,
  )
  mesh = pyrender.Mesh.from_trimesh(tri, material=material, smooth=False)
  bounds = (np.array(tri.bounds[0], dtype=float), np.array(tri.bounds[1], dtype=float))

  # Parse samples
  samples = []
  sample_duration = 0.0
  if(spec.csv_path):
    prefix = spec.csv_path[6:-8]
    samples = samples_from_csv(spec.csv_path, prefix)
    sample_duration = samples[len(samples)-1].time_s

  part = Part(spec, samples, sample_duration, mesh, bounds)
  return part

def main() -> int:
  parts = [part_from_spec(spec) for spec in PART_SPECS]
  for p in parts: print(p)

  # Create scene
  bg_rgb = (16, 16, 16)
  scene = pyrender.Scene(
    bg_color=np.array([bg_rgb[0], bg_rgb[1], bg_rgb[2], 255], dtype=float) / 255.0,
    ambient_light=np.array([0.22, 0.22, 0.22], dtype=float),
  )

  # Create camera
  center = np.array([0,0,0])

  eye = center + np.array([1.3, 2.0, 1.15], dtype=float)
  camera_pose = make_look_at(eye, center, np.array([0.0,0.0,1.0], dtype=float))
  camera = pyrender.PerspectiveCamera(yfov=math.radians(90), znear=0.1, zfar=10000.0)

  scene.add(camera, pose=camera_pose)
  for part in parts:
    pose = np.eye(4, dtype=float)
    if len(part.samples) > 0:
      rest_pos = part.samples[0].pos_mm
      rest_rot = part.samples[0].angles_deg
      tr_mat4 = mat4_from_translate(rest_pos)
      rot_mat4 = mat4_from_euler("313", rest_rot, True)
      pose = tr_mat4 @ rot_mat4
      # pose = tr_mat4
    scene.add(part.mesh, pose=pose)

  viewer = pyrender.Viewer(
    scene,
    viewport_size=(300,300),
    run_in_thread=True,
    use_raymond_lighting=True,
    use_direct_lighting=False,
    refresh_rate=60,
  )

  refresh_rate = 60.0
  while viewer.is_active:
    time.sleep(min(1.0 / refresh_rate, 0.05))

  return 0

if __name__ == "__main__":
  sys.exit(main())
