"""Integration checks run in Isaac Sim; use temporary project files only."""
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace


def run_workflow_checks(app, world, drone, backend, metadata, root, logs):
    import numpy as np
    from pxr import Usd, UsdGeom, Gf
    from flight_ui import FlightUI
    from scene_storage import populate_stage
    checks = []
    temp = tempfile.TemporaryDirectory(prefix='warehouse_workflow_')
    project = Path(temp.name)/'edited_warehouse.usda'
    ui = FlightUI(world, drone, backend, metadata.copy(), project, logs)

    def verify(condition, name):
        assert condition, name
        checks.append(name)
        print('WORKFLOW_CHECK', name, flush=True)

    def action(name):
        ui.request_action(name)
        ui.tick()
        app.update()
        ui.tick()

    def frames(count=12):
        for _ in range(count):
            ui.tick()
            world.step(render=True)

    def physics(seconds):
        for _ in range(round(seconds*120)):
            backend.set_velocity_command(0, 0, 0, 0)
            world.step(render=False)

    action('save')
    verify(not project.exists(), 'save_while_playing_is_rejected')
    action('stop')
    frames()
    verify(world.is_stopped(), 'stop_remains_stopped_across_frames')
    edit = UsdGeom.Cube.Define(world.stage, '/World/UserEditProof')
    edit.CreateSizeAttr(1.25)
    edit.AddTranslateOp().Set(Gf.Vec3d(30, 20, 1.5))
    action('save')
    verify(project.exists(), 'save_stopped_scene')
    edit.GetSizeAttr().Set(2.5)
    action('save')
    saved = Usd.Stage.Open(str(project))
    verify(saved.GetPrimAtPath('/World/UserEditProof').GetAttribute('size').Get() == 2.5,
           'saved_geometry_edit_survives_reopen')
    backups = sorted((project.parent/'backups').glob('*.usda'))
    verify(bool(backups), 'previous_scene_is_backed_up')
    old = Usd.Stage.Open(str(backups[-1]))
    verify(old.GetPrimAtPath('/World/UserEditProof').GetAttribute('size').Get() == 1.25,
           'backup_contains_previous_geometry')
    before_bytes = project.read_bytes()
    loaded = Usd.Stage.CreateInMemory()
    populate_stage(loaded, project)
    verify(loaded.GetPrimAtPath('/World/UserEditProof').GetAttribute('size').Get() == 2.5,
           'startup_loader_preserves_geometry_edit')
    verify(project.read_bytes() == before_bytes, 'startup_loader_does_not_write_project')
    verify(not loaded.GetPrimAtPath('/World/Drone'), 'startup_excludes_saved_runtime_drone')
    action('defaults')
    verify(world.is_stopped(), 'restore_defaults_does_not_start_simulation')
    verify(world.stage.GetPrimAtPath('/World/UserEditProof').GetAttribute('size').Get() == 2.5,
           'restore_defaults_keeps_objects_outside_warehouse')
    action('reset')
    verify(world.is_stopped(), 'reset_preserves_stopped_state')
    # Isaac's native Play follows the same reinitialization route as our button.
    world.play()
    app.update()
    ui.tick()
    verify(world.is_playing(), 'native_play_reinitializes_after_stop')
    backend.takeoff(2)
    physics(8)
    verify(abs(drone.state.position[2]-2) < 0.25, 'takeoff_after_stop_and_play')
    action('pause')
    position = np.asarray(drone.state.position).copy()
    frames()
    verify(not world.is_playing() and not world.is_stopped(), 'pause_remains_paused')
    verify(np.linalg.norm(drone.state.position-position) < 1e-5, 'pause_holds_exact_position')
    verify(backend.mode not in ('GROUNDED', 'FAULT'), 'pause_preserves_flight_controller')
    action('play')
    physics(3)
    verify(abs(drone.state.position[2]-position[2]) < 0.15 and not backend.fault,
           'resume_continues_airborne_without_reset')
    action('pause')
    action('reset')
    verify(not world.is_playing() and not world.is_stopped(), 'reset_preserves_paused_state')
    action('play')
    action('reset')
    verify(world.is_playing(), 'reset_preserves_playing_state')
    verify(np.linalg.norm(np.asarray(drone.state.position)[:2]-[-19, 0]) < 0.1,
           'reset_returns_drone_to_launch_pad')

    # Exercise the same event handler and raw-key polling code with explicit
    # focus/input states; actual native focus is checked separately in the GUI.
    real_window, real_viewport, real_pressed = ui._appwindow, ui.viewport_focused, ui.pressed
    focus = {'app':True, 'viewport':True}
    down = set()
    ui._appwindow = SimpleNamespace(is_focused=lambda:focus['app'])
    ui.viewport_focused = lambda:focus['viewport']
    ui.pressed = lambda key:key in down
    def key(name):
        event = SimpleNamespace(input=SimpleNamespace(name=name),
                                type=ui.carb.input.KeyboardEventType.KEY_PRESS)
        return ui.key_event(event)
    ui.keyboard_enabled = False
    ui.poll_command()
    key('T')
    verify(backend.mode == 'GROUNDED', 'keyboard_disabled_ignores_takeoff')
    ui.keyboard_enabled = True
    focus['viewport'] = False
    down.add('W')
    verify(ui.poll_command() == (0,0,0,0), 'editor_focus_blocks_raw_movement')
    key('T'); key('R')
    verify(backend.mode == 'GROUNDED' and ui.pending_action is None,
           'editor_typing_does_not_takeoff_or_reset')
    focus['viewport'] = True
    verify(ui.poll_command() == (0,0,0,0), 'held_editor_key_requires_release_after_focus_return')
    down.clear()
    ui.poll_command()
    down.add('LEFT_CONTROL')
    key('T')
    verify(backend.mode == 'GROUNDED', 'control_chord_does_not_trigger_takeoff')
    ui.poll_command()
    down.clear()
    ui.poll_command()
    key('T')
    verify(backend.mode == 'TAKEOFF', 'fresh_viewport_takeoff_key_is_accepted')
    down.add('W')
    verify(ui.poll_command()[0] == 1.5, 'fresh_viewport_movement_key_is_accepted')
    focus['app'] = False
    verify(ui.poll_command() == (0,0,0,0), 'application_focus_loss_clears_command')
    ui._appwindow, ui.viewport_focused, ui.pressed = real_window, real_viewport, real_pressed
    action('stop')
    frames()
    result = {'ok':True, 'checks':checks, 'check_count':len(checks),
              'note':'Temporary scene only. Input events use controlled focus states; native UI tested separately.'}
    (logs/'workflow_result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print('WORKFLOW_PASS', flush=True)
    ui.window.visible = False
    # Drop USD file handles before TemporaryDirectory removes its own contents.
    saved = old = loaded = None
    temp.cleanup()
