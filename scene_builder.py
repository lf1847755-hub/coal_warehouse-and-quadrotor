"""Small, fully local coal warehouse for Isaac Sim 5.1 / Pegasus.

Call build_scene(stage) after SimulationApp has started.  The builder only
authors USD and never starts Kit, opens assets, or configures the flight stack.
All dimensions are metres; +Z is up and the clear flight aisle follows +X.
"""

from __future__ import annotations

import math
import random

from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade


ROOT = "/World/CoalWarehouse"


def _material(stage, name, color, roughness=0.8, metallic=0.0):
    material = UsdShade.Material.Define(stage, f"{ROOT}/Materials/{name}")
    shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Surface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _finish(geom, material, collision=False):
    prim = geom.GetPrim()
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    return geom


def _box(stage, path, position, size, material, collision=True, rotate_x=0.0):
    geom = UsdGeom.Cube.Define(stage, f"{ROOT}/{path}")
    geom.CreateSizeAttr(1.0)
    geom.AddTranslateOp().Set(Gf.Vec3d(*position))
    if rotate_x:
        geom.AddRotateXOp().Set(float(rotate_x))
    geom.AddScaleOp().Set(Gf.Vec3f(*size))
    return _finish(geom, material, collision)


def _beam(stage, path, start, end, width, material):
    start, end = Gf.Vec3d(*start), Gf.Vec3d(*end)
    delta = end - start
    geom = UsdGeom.Cube.Define(stage, f"{ROOT}/{path}")
    geom.CreateSizeAttr(1.0)
    geom.AddTranslateOp().Set((start + end) / 2.0)
    rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), delta.GetNormalized())
    geom.AddOrientOp().Set(Gf.Quatf(rotation.GetQuat()))
    geom.AddScaleOp().Set(Gf.Vec3f(width, width, delta.GetLength()))
    return _finish(geom, material, True)


def _disc(stage, path, center, radius, height, material, collision=False):
    geom = UsdGeom.Cylinder.Define(stage, f"{ROOT}/{path}")
    geom.CreateAxisAttr(UsdGeom.Tokens.z)
    geom.CreateRadiusAttr(radius)
    geom.CreateHeightAttr(height)
    geom.AddTranslateOp().Set(Gf.Vec3d(*center))
    return _finish(geom, material, collision)


def _pile(stage, number, center, rx, ry, height, materials):
    """Closed triangulated heap with bounded, irregular concentric contours.

    Cone-like falloff gives an approximately 34--39 degree repose slope.
    The top is slightly offset and every ring has angular perturbations, so
    heaps retain a natural broken silhouette without texture dependencies.
    """
    rng = random.Random(20261023 + number * 97)
    segments = 40
    levels = (1.0, 0.83, 0.65, 0.47, 0.29, 0.12)
    phase = rng.random() * math.tau
    # One smooth radial outline shared by all levels avoids self-intersection.
    outline = [
        1.0 + 0.055 * math.sin(3 * math.tau * j / segments + phase)
        + 0.035 * math.sin(7 * math.tau * j / segments - phase)
        for j in range(segments)
    ]
    points = []
    for ring, radius in enumerate(levels):
        offset = 1.0 - radius
        for j in range(segments):
            angle = math.tau * j / segments
            radial = radius * outline[j] * (1.0 + rng.uniform(-0.018, 0.018))
            # No perturbation at the base: every boundary vertex meets ground.
            z = height * (1.0 - radius)
            if ring:
                z += rng.uniform(-0.065, 0.065)
            points.append(Gf.Vec3f(
                rx * radial * math.cos(angle) + 0.33 * offset,
                ry * radial * math.sin(angle) - 0.19 * offset,
                max(0.0, z),
            ))
    top = len(points)
    points.append(Gf.Vec3f(0.33, -0.19, height))
    bottom = len(points)
    points.append(Gf.Vec3f(0, 0, 0))
    faces = []
    for ring in range(len(levels) - 1):
        for j in range(segments):
            a = ring * segments + j
            b = ring * segments + (j + 1) % segments
            c = (ring + 1) * segments + (j + 1) % segments
            d = (ring + 1) * segments + j
            # Alternate diagonals so faceting does not create obvious spirals.
            if (ring + j) % 2:
                faces.extend(((a, b, d), (b, c, d)))
            else:
                faces.extend(((a, b, c), (a, c, d)))
    for j in range(segments):
        faces.append(((len(levels) - 1) * segments + j,
                      (len(levels) - 1) * segments + (j + 1) % segments, top))
        faces.append((bottom, (j + 1) % segments, j))

    geom = UsdGeom.Mesh.Define(stage, f"{ROOT}/CoalPiles/Pile_{number:02d}")
    geom.CreatePointsAttr(points)
    geom.CreateFaceVertexCountsAttr([3] * len(faces))
    geom.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    geom.CreateDoubleSidedAttr(False)
    geom.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], 0.0))
    geom.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(points))
    _finish(geom, materials[0], True)
    UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("none")
    # Four subtly different rough coal materials vary by face without images.
    # Family type "partition" means each triangle belongs to exactly one subset.
    choices = [rng.choices(range(4), weights=(45, 30, 18, 7))[0] for _ in faces]
    binding = UsdShade.MaterialBindingAPI(geom.GetPrim())
    binding.SetMaterialBindSubsetsFamilyType(UsdGeom.Tokens.partition)
    for index, material in enumerate(materials):
        subset = binding.CreateMaterialBindSubset(
            f"CoalTone{index}", [i for i, choice in enumerate(choices) if choice == index]
        )
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material)
    return {
        "path": str(geom.GetPath()), "center": [center[0], center[1], height / 2],
        "base_radii": [rx, ry], "height": height,
        "vertices": len(points), "triangles": len(faces),
        "collision": "static triangle mesh",
    }


def _arrow(stage, name, x, y, material):
    geom = UsdGeom.Mesh.Define(stage, f"{ROOT}/Markings/{name}")
    # Flat +X arrow. It is paint, so it must not be a physical obstacle.
    geom.CreatePointsAttr([
        (-0.65, -0.10, 0.007), (0.25, -0.10, 0.007),
        (0.25, -0.32, 0.007), (0.8, 0, 0.007),
        (0.25, 0.32, 0.007), (0.25, 0.10, 0.007), (-0.65, 0.10, 0.007),
    ])
    geom.CreateFaceVertexCountsAttr([3, 3, 3])
    geom.CreateFaceVertexIndicesAttr([0, 1, 5, 0, 5, 6, 2, 3, 4])
    geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    geom.AddTranslateOp().Set(Gf.Vec3d(x, y, 0))
    _finish(geom, material)


def build_scene(stage, include_roof=True):
    """Build/replace /World/CoalWarehouse and return JSON-compatible metadata.

    Does not remove /World, create a PhysicsScene, add a drone, set time steps,
    save the stage or start the simulator. Caller owns these integration steps.
    include_roof=False omits only roof sheets; the steel structure remains.
    """
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")
    if not stage.GetDefaultPrim():
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    if stage.GetPrimAtPath(ROOT):
        stage.RemovePrim(ROOT)
    root = UsdGeom.Xform.Define(stage, ROOT)
    root.GetPrim().SetCustomDataByKey("description", "Open coal warehouse; local geometry; metres; Z-up")
    for group in ("Materials", "Structure", "Roof", "CoalPiles", "Markings", "TakeoffPad", "Lights"):
        UsdGeom.Xform.Define(stage, f"{ROOT}/{group}")

    concrete = _material(stage, "Concrete", (0.32, 0.35, 0.38), 0.96)
    aisle = _material(stage, "ClearAisle", (0.43, 0.46, 0.48), 0.94)
    steel = _material(stage, "BlueSteel", (0.075, 0.19, 0.25), 0.5, 0.35)
    roof = _material(stage, "RoofSheet", (0.27, 0.38, 0.42), 0.67, 0.25)
    orange = _material(stage, "SafetyOrange", (0.95, 0.35, 0.055), 0.7)
    yellow = _material(stage, "LaneYellow", (0.98, 0.69, 0.12), 0.75)
    white = _material(stage, "PadWhite", (0.9, 0.93, 0.93), 0.8)
    pad = _material(stage, "PadBlue", (0.055, 0.29, 0.39), 0.92)
    coal = [
        _material(stage, f"Coal{i}", tuple(c * 0.22 for c in color), roughness=0.96 - i * 0.015)
        for i, color in enumerate(((0.034, 0.041, 0.047), (0.044, 0.051, 0.058),
                                   (0.052, 0.060, 0.068), (0.064, 0.071, 0.078)))
    ]

    # The base slab is the sole collision ground; painted patches avoid seams.
    _box(stage, "Ground", (0, 0, -0.18), (64, 44, 0.36), concrete)
    _box(stage, "Markings/Aisle", (0, 0, 0.002), (50, 8.8, 0.004), aisle, False)
    for side in (-1, 1):
        _box(stage, f"Markings/LaneEdge_{'S' if side < 0 else 'N'}", (0, side * 4.4, 0.008),
             (48, 0.12, 0.012), yellow, False)
        _box(stage, f"Markings/OuterEdge_{'S' if side < 0 else 'N'}", (0, side * 15.8, 0.008),
             (49, 0.10, 0.012), yellow, False)
    for i, x in enumerate((-12, -4, 4, 12, 20)):
        _arrow(stage, f"Direction_{i:02d}", x, 0, yellow)
    for i, x in enumerate((-24, 24)):
        for j in range(9):
            _box(stage, f"Markings/Gate_{i}_{j}", (x, -4 + j, 0.008),
                 (0.16, 0.5, 0.012), white, False)

    # Open sides and two full-width end openings; posts are outside flight lanes.
    for bay, x in enumerate((-24, -12, 0, 12, 24)):
        for side in (-1, 1):
            suffix = f"{bay}_{'S' if side < 0 else 'N'}"
            y = 15 * side
            _box(stage, f"Structure/Foot_{suffix}", (x, y, 0.2), (0.78, 0.78, 0.4), concrete)
            _box(stage, f"Structure/Post_{suffix}", (x, y, 4.1), (0.22, 0.28, 7.8), steel)
            _box(stage, f"Structure/Guard_{suffix}", (x, y, 0.85), (0.31, 0.37, 0.9), orange)
            _beam(stage, f"Structure/Rafter_{suffix}", (x, y, 8), (x, 0, 9.2), 0.18, steel)
            _beam(stage, f"Structure/Brace_{suffix}", (x, y, 6.1), (x, y - side * 2.2, 8.18), 0.12, steel)
        _beam(stage, f"Structure/Tie_{bay}", (x, -15, 7.9), (x, 15, 7.9), 0.12, steel)
        # Short vertical king post and web diagonals retain a recognisable truss.
        _beam(stage, f"Structure/KingPost_{bay}", (x, 0, 7.9), (x, 0, 9.2), 0.1, steel)
        for side in (-1, 1):
            _beam(stage, f"Structure/Web_{bay}_{'S' if side < 0 else 'N'}",
                  (x, side * 7.5, 7.9), (x, 0, 9.2), 0.09, steel)
    for side in (-1, 1):
        suffix = "S" if side < 0 else "N"
        for line, y in enumerate((15 * side, 8 * side)):
            z = 9.2 - abs(y) * 0.08
            _beam(stage, f"Structure/Purlin_{suffix}_{line}", (-24, y, z), (24, y, z), 0.16, steel)
        if include_roof:
            # Two seven-metre strips leave a broad central roof opening.
            _box(stage, f"Roof/Sheet_{suffix}", (0, side * 11.5, 8.32),
                 (48.6, 7.2, 0.055), roof, True, -side * math.degrees(math.atan(0.08)))

    piles = []
    for row, y in enumerate((-9.9, 9.9)):
        for col, x in enumerate((-16, -5.3, 5.3, 16)):
            number = row * 4 + col + 1
            radius_x = (4.1, 4.4, 4.2, 4.35)[col]
            radius_y = (3.9, 4.05, 3.8, 4.0)[(col + row) % 4]
            height = (3.2, 3.55, 3.35, 3.4)[(col + row) % 4]
            piles.append(_pile(stage, number, (x, y), radius_x, radius_y, height, coal))

    # Paint lies on the same physical slab, so landing has no mesh seam.
    _disc(stage, "TakeoffPad/OuterRing", (-19, 0, 0.010), 1.8, 0.006, white)
    _disc(stage, "TakeoffPad/BlueCircle", (-19, 0, 0.014), 1.64, 0.006, pad)
    for name, position, size in (
        ("HLeft", (-19.45, 0, 0.020), (0.16, 1.35, 0.006)),
        ("HRight", (-18.55, 0, 0.020), (0.16, 1.35, 0.006)),
        ("HCross", (-19, 0, 0.020), (0.9, 0.16, 0.006)),
    ):
        _box(stage, f"TakeoffPad/{name}", position, size, white, False)

    sun = UsdLux.DistantLight.Define(stage, f"{ROOT}/Lights/Sun")
    sun.CreateIntensityAttr(1500)
    sun.CreateAngleAttr(1.0)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.86))
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-38, -25, -30))
    sky = UsdLux.DomeLight.Define(stage, f"{ROOT}/Lights/SkyFill")
    sky.CreateIntensityAttr(300)
    sky.CreateColorAttr(Gf.Vec3f(0.70, 0.81, 1.0))

    return {
        "root_path": ROOT,
        "units": "metres", "up_axis": "Z",
        "warehouse_dimensions": [48.0, 30.0, 9.2],
        "slab_bounds": [[-32, -22, -0.36], [32, 22, 0]],
        "start_position": [-19.0, 0.0, 1.5],
        "start_yaw_degrees": 0.0,
        "clear_main_aisle": {"x": [-24, 24], "y": [-4.4, 4.4], "height": 7.7},
        "suggested_flight_bounds": [[-25, -13.5, 0.35], [25, 13.5, 6.5]],
        "takeoff_pad_center": [-19.0, 0.0, 0.0],
        "pile_count": len(piles), "piles": piles,
        "pile_triangles": sum(pile["triangles"] for pile in piles),
        "roof_path": f"{ROOT}/Roof", "roof_included": bool(include_roof),
        "external_assets": [],
        "recommended_camera_eye": [-34.0, -32.0, 19.0],
        "recommended_camera_target": [1.0, 0.0, 1.6],
    }
