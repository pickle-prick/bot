#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import importlib
import importlib.util
import math
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class PartSpec:
    mesh_name: str
    obj_name: str
    csv_name: str | None
    csv_prefix: str | None
    parent_mesh: str | None
    color_rgb: tuple[int, int, int]


@dataclass(frozen=True)
class PoseSample:
    time_s: float
    pos_mm: tuple[float, float, float]
    angles_deg: tuple[float, float, float]


PARTS: tuple[PartSpec, ...] = (
    PartSpec("Base-1_Mesh", "Base-1_Mesh.OBJ", None, None, None, (16, 16, 16)),
    PartSpec("Link_1-1_Mesh", "Link_1-1_Mesh.OBJ", "link_1_res.csv", "link_1", "Base-1_Mesh", (210, 60, 60)),
    PartSpec("Link_2-2_Mesh", "Link_2-2_Mesh.OBJ", "link_2_res.csv", "link_2", "Link_1-1_Mesh", (235, 170, 55)),
    PartSpec("Link_3-1_Mesh", "Link_3-1_Mesh.OBJ", "link_3_res.csv", "link_3", "Link_2-2_Mesh", (65, 135, 220)),
    PartSpec("Link_4-1_Mesh", "Link_4-1_Mesh.OBJ", "link_4_res.csv", "link_4", "Link_3-1_Mesh", (90, 185, 120)),
    PartSpec("Link_5-1_Mesh", "Link_5-1_Mesh.OBJ", "link_5_res.csv", "link_5", "Link_4-1_Mesh", (125, 90, 205)),
    PartSpec("Flange-1_Mesh", "Flange-1_Mesh.OBJ", "flange_res.csv", "flange", "Link_5-1_Mesh", (60, 170, 170)),
    PartSpec("Tool-1_Mesh", "Tool-1_Mesh.OBJ", None, None, "Flange-1_Mesh", (80, 80, 80)),
)

# The OBJ exports use the same X axis as Datasmith, but their Y/Z axes are
# swapped. Remap the geometry once on load so the rest transforms assemble
# correctly.
OBJ_TO_SCENE_BASIS = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)


def lazy_import(module_name: str):
    module = importlib.util.find_spec(module_name)
    if module is None:
        raise RuntimeError(module_name)
    return importlib.import_module(module_name)


def patch_pyrender_viewer_mouse_drag(pyrender) -> None:
    viewer_cls = getattr(pyrender, "Viewer", None)
    if viewer_cls is None or getattr(viewer_cls, "_robot_sequence_drag_patch", False):
        return

    original_on_mouse_drag = viewer_cls.on_mouse_drag

    def safe_on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        trackball = getattr(self, "_trackball", None)
        if trackball is None or not hasattr(trackball, "_pdown"):
            return None
        return original_on_mouse_drag(self, x, y, dx, dy, buttons, modifiers)

    viewer_cls.on_mouse_drag = safe_on_mouse_drag
    viewer_cls._robot_sequence_drag_patch = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Render the robot OBJ sequence using the CSV pose tracks. "
            "The script assumes the CSV positions are absolute in the robot base space, "
            "converts mm->cm, and maps source XYZ to Unreal-style XZY."
        )
    )
    parser.add_argument("--objs-dir", type=Path, default=Path("objs"), help="Directory that contains the OBJ files.")
    parser.add_argument("--robot-dir", type=Path, default=Path("Robot"), help="Directory that contains the CSVs and Robot.udatasmith.")
    parser.add_argument("--datasmith", type=Path, default=None, help="Override the Robot.udatasmith path.")
    parser.add_argument("--output-dir", type=Path, default=Path("render_frames"), help="Output directory for the rendered image sequence.")
    parser.add_argument("--width", type=int, default=1280, help="Image width.")
    parser.add_argument("--height", type=int, default=720, help="Image height.")
    parser.add_argument("--start-frame", type=int, default=0, help="First CSV frame index to render.")
    parser.add_argument("--end-frame", type=int, default=None, help="End frame index, exclusive. Defaults to the CSV length.")
    parser.add_argument("--frame-step", type=int, default=1, help="Render every Nth frame.")
    parser.add_argument("--rotation-order", default="313", help="Euler order from the CSV. Defaults to 313.")
    parser.add_argument(
        "--euler-mode",
        choices=("intrinsic", "extrinsic"),
        default="intrinsic",
        help="Euler interpretation. Robotics exports usually mean intrinsic.",
    )
    parser.add_argument(
        "--transform-mode",
        choices=("serial", "world"),
        default="serial",
        help="serial computes local parent-child transforms from the absolute CSV poses; world applies absolute poses directly.",
    )
    parser.add_argument(
        "--reflect-for-render",
        action="store_true",
        default=True,
        help="Apply a global Y reflection before rendering so the scene is right-handed for OpenGL-based renderers.",
    )
    parser.add_argument(
        "--no-reflect-for-render",
        dest="reflect_for_render",
        action="store_false",
        help="Disable the global Y reflection.",
    )
    parser.add_argument("--yfov-deg", type=float, default=35.0, help="Perspective camera vertical field of view in degrees.")
    parser.add_argument("--samples-for-bounds", type=int, default=5, help="How many frames to sample when fitting the camera.")
    parser.add_argument("--light-intensity", type=float, default=6.0, help="Directional light intensity.")
    parser.add_argument("--bg", default="26,28,36", help="Background color as R,G,B in 0..255.")
    parser.add_argument("--viewer", action="store_true", help="Open an interactive viewer instead of writing a PPM sequence.")
    parser.add_argument("--playback-speed", type=float, default=1.0, help="Playback speed multiplier for --viewer mode.")
    parser.add_argument("--viewer-refresh-rate", type=float, default=60.0, help="Viewer redraw rate in Hz.")
    parser.add_argument(
        "--no-loop-playback",
        dest="loop_playback",
        action="store_false",
        help="Play the selected frame range once in viewer mode instead of looping.",
    )
    parser.set_defaults(loop_playback=True)
    return parser.parse_args()


def parse_rgb(text: str) -> tuple[int, int, int]:
    values = [int(piece.strip()) for piece in text.split(",")]
    if len(values) != 3:
        raise ValueError(f"expected R,G,B, got {text!r}")
    return tuple(max(0, min(255, value)) for value in values)  # type: ignore[return-value]


def parse_transform_element(transform_elem, np):
    tx = float(transform_elem.attrib["tx"])
    ty = float(transform_elem.attrib["ty"])
    tz = float(transform_elem.attrib["tz"])
    qx = float(transform_elem.attrib["qx"])
    qy = float(transform_elem.attrib["qy"])
    qz = float(transform_elem.attrib["qz"])
    qw = float(transform_elem.attrib["qw"])

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    rot = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rot
    matrix[:3, 3] = np.array([tx, ty, tz], dtype=float)
    return matrix


def load_rest_transforms(datasmith_path: Path, np):
    tree = ET.parse(datasmith_path)
    scene_root = tree.getroot()
    actor = scene_root.find(".//Actor[@name='HSR-JR603-570']")
    if actor is None:
        raise ValueError(f"Could not find Actor[name='HSR-JR603-570'] in {datasmith_path}")

    root_transform_elem = actor.find("Transform")
    actor_root = parse_transform_element(root_transform_elem, np) if root_transform_elem is not None else np.eye(4, dtype=float)

    rest_by_mesh: dict[str, object] = {}
    children = actor.find("children")
    if children is None:
        raise ValueError(f"No actor children found in {datasmith_path}")

    for actor_mesh in children.findall("ActorMesh"):
        mesh_ref = actor_mesh.find("mesh")
        transform_elem = actor_mesh.find("Transform")
        if mesh_ref is None or transform_elem is None:
            continue
        mesh_name = mesh_ref.attrib["name"]
        rest_by_mesh[mesh_name] = parse_transform_element(transform_elem, np)

    return actor_root, rest_by_mesh


def load_pose_series(csv_path: Path, prefix: str) -> list[PoseSample]:
    rows: list[PoseSample] = []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(
                PoseSample(
                    time_s=float(row["TIME"]),
                    pos_mm=(
                        float(row[f"{prefix}_ox"]),
                        float(row[f"{prefix}_oy"]),
                        float(row[f"{prefix}_oz"]),
                    ),
                    angles_deg=(
                        float(row[f"{prefix}_e1"]),
                        float(row[f"{prefix}_e2"]),
                        float(row[f"{prefix}_e3"]),
                    ),
                )
            )
    if not rows:
        raise ValueError(f"No pose rows found in {csv_path}")
    return rows


def rotation_axis_matrix(axis_digit: str, angle_deg: float, np):
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


def euler_matrix(order: str, angles_deg: tuple[float, float, float], intrinsic: bool, np):
    if len(order) != 3:
        raise ValueError(f"Rotation order must have length 3, got {order!r}")

    a1, a2, a3 = angles_deg
    pieces = [
        rotation_axis_matrix(order[0], a1, np),
        rotation_axis_matrix(order[1], a2, np),
        rotation_axis_matrix(order[2], a3, np),
    ]
    if intrinsic:
        result = np.eye(3, dtype=float)
        for piece in pieces:
            result = result @ piece
        return result

    result = np.eye(3, dtype=float)
    for piece in reversed(pieces):
        result = result @ piece
    return result


def csv_rotation_to_scene_space(angles_deg: tuple[float, float, float], order: str, intrinsic: bool, np):
    source_rot = euler_matrix(order, angles_deg, intrinsic=intrinsic, np=np)
    axis_swap = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=float,
    )
    return axis_swap @ source_rot @ axis_swap.T


def csv_position_to_scene_space(pos_mm: tuple[float, float, float], np):
    x_mm, y_mm, z_mm = pos_mm
    return np.array([x_mm * 0.1, z_mm * 0.1, y_mm * 0.1], dtype=float)


def compose_transform(rot3, translation3, np):
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rot3
    matrix[:3, 3] = translation3
    return matrix


def rigid_inverse(matrix4, np):
    rot = matrix4[:3, :3]
    trans = matrix4[:3, 3]
    inv = np.eye(4, dtype=float)
    inv[:3, :3] = rot.T
    inv[:3, 3] = -(rot.T @ trans)
    return inv


def build_world_frames(parts, rest_by_mesh, pose_tracks, order: str, intrinsic: bool, np):
    frame_count = min(len(track) for track in pose_tracks.values())
    first_rot_by_mesh = {
        mesh_name: csv_rotation_to_scene_space(track[0].angles_deg, order=order, intrinsic=intrinsic, np=np)
        for mesh_name, track in pose_tracks.items()
    }
    tool_rest_local = rigid_inverse(rest_by_mesh["Flange-1_Mesh"], np) @ rest_by_mesh["Tool-1_Mesh"]

    frames: list[tuple[float, dict[str, object]]] = []
    for frame_idx in range(frame_count):
        world: dict[str, object] = {
            "Base-1_Mesh": rest_by_mesh["Base-1_Mesh"],
        }

        for part in parts:
            if part.csv_name is None:
                continue
            sample = pose_tracks[part.mesh_name][frame_idx]
            current_rot = csv_rotation_to_scene_space(sample.angles_deg, order=order, intrinsic=intrinsic, np=np)
            delta_rot = current_rot @ first_rot_by_mesh[part.mesh_name].T
            rest_rot = rest_by_mesh[part.mesh_name][:3, :3]
            rest_translation = rest_by_mesh[part.mesh_name][:3, 3]

            matrix = compose_transform(
                delta_rot @ rest_rot,
                csv_position_to_scene_space(sample.pos_mm, np=np),
                np=np,
            )

            # Link translations already match the frame-0 Datasmith positions in cm after x,z,y mapping.
            # The rest translation is only useful as a sanity check during debugging.
            if frame_idx == 0:
                delta_translation = matrix[:3, 3] - rest_translation
                if float(np.linalg.norm(delta_translation)) > 0.5:
                    print(
                        f"warning: frame-0 position mismatch for {part.mesh_name}: "
                        f"{delta_translation.round(4).tolist()} cm",
                        file=sys.stderr,
                    )
            world[part.mesh_name] = matrix

        world["Tool-1_Mesh"] = world["Flange-1_Mesh"] @ tool_rest_local
        frame_time = pose_tracks["Link_1-1_Mesh"][frame_idx].time_s
        frames.append((frame_time, world))
    return frames


def build_local_frame(world_frame, parts, np):
    local_frame: dict[str, object] = {}
    for part in parts:
        world_matrix = world_frame[part.mesh_name]
        if part.parent_mesh is None:
            local_frame[part.mesh_name] = world_matrix
            continue
        parent_world = world_frame[part.parent_mesh]
        local_frame[part.mesh_name] = rigid_inverse(parent_world, np) @ world_matrix
    return local_frame


def transformed_bbox_corners(local_bounds, matrix4, np):
    bound_min, bound_max = local_bounds
    corners = np.array(
        [
            [bound_min[0], bound_min[1], bound_min[2], 1.0],
            [bound_min[0], bound_min[1], bound_max[2], 1.0],
            [bound_min[0], bound_max[1], bound_min[2], 1.0],
            [bound_min[0], bound_max[1], bound_max[2], 1.0],
            [bound_max[0], bound_min[1], bound_min[2], 1.0],
            [bound_max[0], bound_min[1], bound_max[2], 1.0],
            [bound_max[0], bound_max[1], bound_min[2], 1.0],
            [bound_max[0], bound_max[1], bound_max[2], 1.0],
        ],
        dtype=float,
    )
    transformed = (matrix4 @ corners.T).T
    return transformed[:, :3]


def sample_indices(frame_count: int, sample_count: int) -> list[int]:
    if frame_count <= 0:
        return []
    if sample_count <= 1 or frame_count == 1:
        return [0]
    if sample_count >= frame_count:
        return list(range(frame_count))
    return sorted({round(i * (frame_count - 1) / (sample_count - 1)) for i in range(sample_count)})


def make_look_at(eye, target, up_hint, np):
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


def write_ppm(path: Path, rgb_image) -> None:
    height, width, channels = rgb_image.shape
    if channels != 3:
        raise ValueError(f"Expected an RGB image, got {rgb_image.shape}")
    with path.open("wb") as handle:
        handle.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        handle.write(rgb_image.astype("uint8", copy=False).tobytes())


def load_meshes(parts, objs_dir: Path, np, trimesh, pyrender):
    meshes = {}
    bounds = {}
    basis = np.array(OBJ_TO_SCENE_BASIS, dtype=float)
    for part in parts:
        obj_path = objs_dir / part.obj_name
        if not obj_path.exists():
            raise FileNotFoundError(obj_path)

        tri = trimesh.load(obj_path, force="mesh", process=False)
        if isinstance(tri, trimesh.Scene):
            tri = trimesh.util.concatenate(tuple(tri.geometry.values()))
        tri = tri.copy()
        tri.apply_transform(basis)

        color = tuple(channel / 255.0 for channel in part.color_rgb) + (1.0,)
        material = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=color,
            metallicFactor=0.0,
            roughnessFactor=0.7,
            doubleSided=True,
        )
        meshes[part.mesh_name] = pyrender.Mesh.from_trimesh(tri, material=material, smooth=False)
        bounds[part.mesh_name] = (
            np.array(tri.bounds[0], dtype=float),
            np.array(tri.bounds[1], dtype=float),
        )
    return meshes, bounds


def add_default_lights(scene, center, radius, intensity, np, pyrender):
    light = pyrender.DirectionalLight(color=np.ones(3, dtype=float), intensity=float(intensity))
    offsets = (
        np.array([radius * 1.8, -radius * 1.6, radius * 1.6], dtype=float),
        np.array([-radius * 1.2, radius * 1.4, radius * 1.1], dtype=float),
        np.array([radius * 0.4, radius * 1.7, radius * 2.0], dtype=float),
    )
    for offset in offsets:
        eye = center + offset
        pose = make_look_at(eye, center, np.array([0.0, 0.0, 1.0], dtype=float), np)
        scene.add(light, pose=pose)


def apply_world_frame(world, nodes, parts, reflect, transform_mode: str, scene, np):
    if transform_mode == "serial":
        local = build_local_frame(world, parts, np=np)
        for part in parts:
            pose = reflect @ local[part.mesh_name] if part.parent_mesh is None else local[part.mesh_name]
            scene.set_pose(nodes[part.mesh_name], pose=pose)
        return

    for part in parts:
        scene.set_pose(nodes[part.mesh_name], pose=reflect @ world[part.mesh_name])


def play_frames_in_viewer(
    scene,
    nodes,
    selected_frames,
    parts,
    reflect,
    transform_mode: str,
    width: int,
    height: int,
    playback_speed: float,
    refresh_rate: float,
    loop_playback: bool,
    np,
    pyrender,
):
    if playback_speed <= 0.0:
        raise ValueError("--playback-speed must be > 0")
    if refresh_rate <= 0.0:
        raise ValueError("--viewer-refresh-rate must be > 0")
    if not selected_frames:
        raise ValueError("No frames selected for viewer playback.")

    _, _, initial_world = selected_frames[0]
    apply_world_frame(initial_world, nodes=nodes, parts=parts, reflect=reflect, transform_mode=transform_mode, scene=scene, np=np)

    viewer = pyrender.Viewer(
        scene,
        viewport_size=(width, height),
        run_in_thread=True,
        use_raymond_lighting=False,
        use_direct_lighting=False,
        refresh_rate=refresh_rate,
        window_title="Robot Sequence Viewer",
    )
    print("viewer mode active: close the window or press q to quit.")

    first_time_s = selected_frames[0][1]
    while viewer.is_active:
        wall_start = time.perf_counter()
        for _frame_idx, time_s, world in selected_frames:
            if not viewer.is_active:
                break

            target_elapsed = (time_s - first_time_s) / playback_speed
            while viewer.is_active:
                remaining = target_elapsed - (time.perf_counter() - wall_start)
                if remaining <= 0.0:
                    break
                time.sleep(min(remaining, 0.005))

            with viewer.render_lock:
                apply_world_frame(
                    world,
                    nodes=nodes,
                    parts=parts,
                    reflect=reflect,
                    transform_mode=transform_mode,
                    scene=scene,
                    np=np,
                )

        if not loop_playback:
            break

    while viewer.is_active:
        time.sleep(min(1.0 / refresh_rate, 0.05))


def main() -> int:
    args = parse_args()
    bg_rgb = parse_rgb(args.bg)

    try:
        np = lazy_import("numpy")
        trimesh = lazy_import("trimesh")
        pyrender = lazy_import("pyrender")
    except RuntimeError as exc:
        print(
            "Missing Python dependency: "
            f"{exc}. Install with: pip install numpy trimesh pyrender \"pyglet<2\"",
            file=sys.stderr,
        )
        return 2

    if not hasattr(np, "infty"):
        setattr(np, "infty", np.inf)
    patch_pyrender_viewer_mouse_drag(pyrender)

    objs_dir = args.objs_dir.resolve()
    robot_dir = args.robot_dir.resolve()
    datasmith_path = args.datasmith.resolve() if args.datasmith else (robot_dir / "Robot.udatasmith").resolve()

    actor_root, rest_by_mesh = load_rest_transforms(datasmith_path, np=np)
    _ = actor_root

    pose_tracks: dict[str, list[PoseSample]] = {}
    for part in PARTS:
        if part.csv_name is None or part.csv_prefix is None:
            continue
        pose_tracks[part.mesh_name] = load_pose_series(robot_dir / part.csv_name, part.csv_prefix)

    world_frames = build_world_frames(
        parts=PARTS,
        rest_by_mesh=rest_by_mesh,
        pose_tracks=pose_tracks,
        order=args.rotation_order,
        intrinsic=args.euler_mode == "intrinsic",
        np=np,
    )
    if not world_frames:
        raise ValueError("No world frames were generated.")

    frame_count = len(world_frames)
    start_frame = max(0, args.start_frame)
    end_frame = frame_count if args.end_frame is None else min(frame_count, args.end_frame)
    if start_frame >= end_frame:
        raise ValueError(f"Invalid frame range: start={start_frame}, end={end_frame}, count={frame_count}")
    if args.frame_step <= 0:
        raise ValueError("--frame-step must be > 0")
    selected_frames = [(frame_idx, *world_frames[frame_idx]) for frame_idx in range(start_frame, end_frame, args.frame_step)]
    if not selected_frames:
        raise ValueError("No frames selected.")

    meshes, mesh_bounds = load_meshes(PARTS, objs_dir=objs_dir, np=np, trimesh=trimesh, pyrender=pyrender)

    reflect = np.eye(4, dtype=float)
    if args.reflect_for_render:
        reflect[1, 1] = -1.0

    sampled_world_indices = sample_indices(frame_count, args.samples_for_bounds)
    scene_min = np.array([float("inf"), float("inf"), float("inf")], dtype=float)
    scene_max = np.array([float("-inf"), float("-inf"), float("-inf")], dtype=float)
    for frame_idx in sampled_world_indices:
        _, world = world_frames[frame_idx]
        for part in PARTS:
            render_world = reflect @ world[part.mesh_name]
            corners = transformed_bbox_corners(mesh_bounds[part.mesh_name], render_world, np=np)
            scene_min = np.minimum(scene_min, corners.min(axis=0))
            scene_max = np.maximum(scene_max, corners.max(axis=0))

    center = 0.5 * (scene_min + scene_max)
    extent = np.maximum(scene_max - scene_min, np.array([1.0, 1.0, 1.0], dtype=float))
    radius = float(np.linalg.norm(extent) * 0.7)
    eye = center + np.array([radius * 1.3, -radius * 2.0, radius * 1.15], dtype=float)
    camera_pose = make_look_at(eye, center, np.array([0.0, 0.0, 1.0], dtype=float), np)

    scene = pyrender.Scene(
        bg_color=np.array([bg_rgb[0], bg_rgb[1], bg_rgb[2], 255], dtype=float) / 255.0,
        ambient_light=np.array([0.22, 0.22, 0.22], dtype=float),
    )

    camera = pyrender.PerspectiveCamera(yfov=math.radians(args.yfov_deg), znear=0.1, zfar=10000.0)
    scene.add(camera, pose=camera_pose)
    add_default_lights(scene, center=center, radius=radius, intensity=args.light_intensity, np=np, pyrender=pyrender)

    nodes = {}
    if args.transform_mode == "serial":
        for part in PARTS:
            parent_node = nodes.get(part.parent_mesh)
            nodes[part.mesh_name] = scene.add(meshes[part.mesh_name], pose=np.eye(4, dtype=float), parent_node=parent_node)
    else:
        for part in PARTS:
            nodes[part.mesh_name] = scene.add(meshes[part.mesh_name], pose=np.eye(4, dtype=float))

    if args.viewer:
        play_frames_in_viewer(
            scene,
            nodes=nodes,
            selected_frames=selected_frames,
            parts=PARTS,
            reflect=reflect,
            transform_mode=args.transform_mode,
            width=args.width,
            height=args.height,
            playback_speed=args.playback_speed,
            refresh_rate=args.viewer_refresh_rate,
            loop_playback=args.loop_playback,
            np=np,
            pyrender=pyrender,
        )
        return 0

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    renderer = pyrender.OffscreenRenderer(viewport_width=args.width, viewport_height=args.height)
    try:
        for output_idx, (frame_idx, time_s, world) in enumerate(selected_frames):
            apply_world_frame(
                world,
                nodes=nodes,
                parts=PARTS,
                reflect=reflect,
                transform_mode=args.transform_mode,
                scene=scene,
                np=np,
            )

            color, _depth = renderer.render(scene)
            output_path = output_dir / f"frame_{output_idx:04d}_src_{frame_idx:04d}_{time_s:07.3f}s.ppm"
            write_ppm(output_path, color)
            print(f"[{output_idx + 1}/{len(selected_frames)}] {output_path.name}")
    finally:
        renderer.delete()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
