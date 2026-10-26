"""Read-only scene loading and explicit, backed-up USD saves.

The saved file contains the editable environment, not the runtime Pegasus
vehicle or the renderer's temporary prims.  Importing this module does not
start Kit; pxr is imported only when a USD operation is requested.
"""

from __future__ import annotations

from datetime import datetime
import os
from pathlib import Path
import shutil
import uuid


RUNTIME_PATHS = ("/World/Drone", "/Render")


def _environment_snapshot(stage):
    from pxr import Usd

    # Flatten resolves referenced layers and anchors relative asset paths to
    # their original layers.  It also includes unsaved/session-layer edits.
    # Work on the snapshot so pruning never modifies the current scene.
    layer = stage.Flatten()
    if layer is None:
        raise RuntimeError("Could not create a USD snapshot")
    snapshot = Usd.Stage.Open(layer)
    for path in RUNTIME_PATHS:
        if snapshot.GetPrimAtPath(path):
            snapshot.RemovePrim(path)
    return layer


def populate_stage(stage, source_path):
    """Load an environment into a fresh stage without writing the source file.

    Call before constructing World/Pegasus.  The current stage's root layer
    must be anonymous so normal editor saves cannot silently write the source.
    References are flattened; the source file remains untouched, including
    when it was produced by an older demo containing /World/Drone.
    """
    from pxr import Usd

    source = Path(source_path).resolve(strict=True)
    if not stage.GetRootLayer().anonymous:
        raise ValueError("Load into an anonymous working stage, not a source file")
    source_stage = Usd.Stage.Open(str(source))
    if source_stage is None:
        raise RuntimeError(f"Could not open scene: {source}")
    layer = _environment_snapshot(source_stage)
    stage.GetRootLayer().TransferContent(layer)
    stage.SetEditTarget(stage.GetRootLayer())
    return {"path": str(source), "loaded": True, "runtime_paths_removed": list(RUNTIME_PATHS)}


def save_scene(stage, path, *, stopped: bool):
    """Save an environment atomically, preserving the previous file in backups.

    The caller must pass the actual stopped state of the simulation.  Paused
    flight is insufficient: Stop first so runtime transforms are restored.
    Failures before replacement leave the previous scene file unchanged.
    """
    if not stopped:
        raise RuntimeError("Stop the simulation before saving the scene")
    destination = Path(path).resolve()
    if destination.suffix.lower() not in (".usd", ".usda", ".usdc"):
        raise ValueError("Scene path must end in .usd, .usda or .usdc")
    destination.parent.mkdir(parents=True, exist_ok=True)
    token = uuid.uuid4().hex[:12]
    temporary = destination.with_name(f".{destination.stem}.{token}.tmp{destination.suffix}")
    backup = None
    backup_temporary = None
    try:
        layer = _environment_snapshot(stage)
        if not layer.Export(str(temporary)):
            raise RuntimeError(f"Could not export scene: {destination}")
        # Flush before publishing the new file.  The temporary file is on the
        # same volume, so os.replace performs an atomic directory entry swap.
        with temporary.open("rb+") as stream:
            os.fsync(stream.fileno())
        if destination.exists():
            backup_dir = destination.parent / "backups"
            backup_dir.mkdir(exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            backup = backup_dir / f"{destination.stem}.{stamp}-{token}{destination.suffix}"
            backup_temporary = backup.with_name(f".{backup.name}.tmp")
            shutil.copy2(destination, backup_temporary)
            with backup_temporary.open("rb+") as stream:
                os.fsync(stream.fileno())
            os.replace(backup_temporary, backup)
        os.replace(temporary, destination)
    finally:
        for candidate in (temporary, backup_temporary):
            if candidate is not None and candidate.exists():
                candidate.unlink()
    return {"path": str(destination), "backup": str(backup) if backup else None}
