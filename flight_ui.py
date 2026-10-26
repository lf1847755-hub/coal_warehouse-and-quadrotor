"""Editor-safe manual flight controls for the local warehouse project."""
import json
import math
import time


class FlightUI:
    MOVEMENT_KEYS = ('W', 'A', 'S', 'D', 'Q', 'E', 'UP', 'DOWN')
    ACTION_KEYS = ('T', 'H', 'L', 'R', 'F', 'O', 'V')

    def __init__(self, world, drone, backend, metadata, project_path, logs):
        import carb
        import omni.ui as ui
        import omni.appwindow
        from omni.kit.viewport.utility import get_active_viewport_window
        self.world, self.drone, self.backend, self.metadata = world, drone, backend, metadata
        self.project_path, self.logs = project_path, logs
        self.carb, self.ui = carb, ui
        self._get_viewport = get_active_viewport_window
        self.camera = 'follow'
        self.keyboard_enabled = False
        self._keys_released = False
        self._was_keyboard_active = False
        self.pending_action = None
        self._last_timeline_state = self.timeline_state()
        self.last_log = 0
        self.frame = 0
        self.last_key = ''
        self.key_count = 0
        self.notice = 'Keyboard OFF. Enable, then click inside the 3D viewport.'
        self._input = carb.input.acquire_input_interface()
        self._appwindow = omni.appwindow.get_default_app_window()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub = self._input.subscribe_to_keyboard_events(self._keyboard, self.key_event)
        self.window = ui.Window('Coal Warehouse - Manual Flight', width=340, height=690)
        self.window.position_x = 920
        self.window.position_y = 70
        for name in ('Content', 'Console'):
            editor_window = ui.Workspace.get_window(name)
            if editor_window:
                editor_window.visible = False
        with self.window.frame:
            with ui.VStack(spacing=5, style={'font_size':16}):
                ui.Label('COAL WAREHOUSE', height=28,
                         style={'font_size':23, 'color':0xFF66D9F2})
                ui.Label('Isaac Sim 5.1 + Pegasus 5.1', height=22)
                with ui.HStack(height=32, spacing=5):
                    self.play_button = ui.Button('Play / Resume', clicked_fn=lambda:self.request_action('play'))
                    self.pause_button = ui.Button('Pause', clicked_fn=lambda:self.request_action('pause'))
                    self.stop_button = ui.Button('Stop / Edit', clicked_fn=lambda:self.request_action('stop'))
                self.keyboard_button = ui.Button('Enable flight keyboard', height=32,
                                                 clicked_fn=self.toggle_keyboard)
                self.key_status = ui.Label('', height=38, word_wrap=True, style={'font_size':14})
                with ui.HStack(height=32, spacing=5):
                    self.takeoff_button = ui.Button('Takeoff [T]', clicked_fn=lambda:self.flight_action('T'))
                    self.hold_button = ui.Button('Hold [H]', clicked_fn=lambda:self.flight_action('H'))
                    self.land_button = ui.Button('Land [L]', clicked_fn=lambda:self.flight_action('L'))
                ui.Label('W / S: forward / backward    A / D: left / right', height=23, style={'font_size':14})
                ui.Label('Q / E: turn    Up / Down: ascend / descend', height=23, style={'font_size':14})
                ui.Label('Release keys to hover. Use English input.', height=23, style={'font_size':14})
                with ui.HStack(height=30, spacing=5):
                    ui.Button('Follow [F]', clicked_fn=lambda:self.set_camera('follow'))
                    ui.Button('Overview [O]', clicked_fn=lambda:self.set_camera('overview'))
                with ui.HStack(height=30, spacing=5):
                    ui.Button('Roof view [V]', clicked_fn=self.toggle_roof)
                    ui.Button('Reset drone [R]', clicked_fn=lambda:self.request_action('reset'))
                ui.Separator(height=5)
                ui.Label('EDIT: Stop -> edit scene -> Save scene', height=23, style={'font_size':14})
                self.save_button = ui.Button('Save scene + backup', height=30,
                                            clicked_fn=lambda:self.request_action('save'))
                self.defaults_button = ui.Button('Restore default warehouse...', height=30,
                                                 clicked_fn=self.confirm_defaults)
                self.status = ui.Label('', height=62, word_wrap=True)
                self.notice_label = ui.Label(self.notice, height=62, word_wrap=True, style={'font_size':14})
                ui.Label('Speed 1.5 m/s  |  Altitude limit 6.5 m', height=22, style={'font_size':14})
        self.update_camera()

    def timeline_state(self):
        if self.world.is_playing():
            return 'playing'
        return 'stopped' if self.world.is_stopped() else 'paused'

    def request_action(self, action):
        self.pending_action = action

    def toggle_keyboard(self):
        self.keyboard_enabled = not self.keyboard_enabled
        self._keys_released = False
        self.backend.set_velocity_command(0, 0, 0, 0)
        if not self.keyboard_enabled and self.backend.mode == 'FLYING':
            self.backend.hold()
        self.notice = ('Click inside the 3D viewport, release all keys, then press T.'
                       if self.keyboard_enabled else 'Keyboard OFF. Scene editor shortcuts are available.')

    def viewport_focused(self):
        viewport = self._get_viewport()
        return bool(viewport and viewport.focused)

    def pressed(self, name):
        key = getattr(self.carb.input.KeyboardInput, name, None)
        return bool(key is not None and self._input.get_keyboard_value(self._keyboard, key) > 0.5)

    def input_context_allowed(self):
        # Editor panels, dialogs, other applications and shortcut chords must
        # never become aircraft commands. Fail closed when no viewport exists.
        modifiers = ('LEFT_CONTROL', 'RIGHT_CONTROL', 'LEFT_ALT', 'RIGHT_ALT',
                     'LEFT_SUPER', 'RIGHT_SUPER', 'LEFT_SHIFT', 'RIGHT_SHIFT')
        return (self.keyboard_enabled and self.world.is_playing()
                and self._appwindow.is_focused() and self.viewport_focused()
                and not any(self.pressed(k) for k in modifiers))

    def poll_command(self):
        allowed = self.input_context_allowed()
        if not allowed:
            self._keys_released = False
        elif not self._keys_released:
            # A W held while typing or switching windows cannot start flying on
            # focus return. Require a complete release before accepting input.
            self._keys_released = not any(self.pressed(k) for k in self.MOVEMENT_KEYS + self.ACTION_KEYS)
        active = allowed and self._keys_released
        if self._was_keyboard_active and not active and self.backend.mode == 'FLYING':
            self.backend.hold()
        self._was_keyboard_active = active
        if not active:
            return (0, 0, 0, 0)
        down = lambda name: float(self.pressed(name))
        return ((down('W')-down('S'))*1.5, (down('A')-down('D'))*1.5,
                (down('UP')-down('DOWN'))*0.7, (down('Q')-down('E'))*0.6)

    def flight_action(self, key):
        if not self.world.is_playing():
            self.notice = 'Press Play / Resume before issuing flight commands.'
            return
        {'T':lambda:self.backend.takeoff(2), 'H':self.backend.hold, 'L':self.backend.land}[key]()

    def key_event(self, event, *unused):
        key_name = getattr(event.input, 'name', str(event.input)).upper()
        if not (self.input_context_allowed() and self._keys_released):
            return True
        if event.type == self.carb.input.KeyboardEventType.KEY_PRESS:
            self.last_key = key_name
            self.key_count += 1
            actions = {'T':lambda:self.flight_action('T'), 'H':lambda:self.flight_action('H'),
                       'L':lambda:self.flight_action('L'), 'R':lambda:self.request_action('reset'),
                       'F':lambda:self.set_camera('follow'), 'O':lambda:self.set_camera('overview'),
                       'V':self.toggle_roof}
            if key_name in actions:
                actions[key_name]()
                return False
        return key_name not in self.MOVEMENT_KEYS + self.ACTION_KEYS

    def set_camera(self, mode):
        self.camera = mode
        self.update_camera()

    def toggle_roof(self):
        from pxr import UsdGeom
        prim = self.world.stage.GetPrimAtPath('/World/CoalWarehouse/Roof')
        if prim:
            attr = UsdGeom.Imageable(prim).GetVisibilityAttr()
            attr.Set('inherited' if attr.Get() == 'invisible' else 'invisible')

    def confirm_defaults(self):
        if not self.world.is_stopped():
            self.notice = 'Stop the simulation before restoring the warehouse.'
            return
        ui = self.ui
        self._confirm_window = ui.Window('Restore default warehouse?', width=440, height=180,
                                        flags=ui.WINDOW_FLAGS_NO_RESIZE)
        with self._confirm_window.frame:
            with ui.VStack(spacing=10):
                ui.Label('Replace the warehouse geometry with the original 8 piles?\n'
                         'The current scene will be saved and backed up first.\n'
                         'Other objects outside CoalWarehouse are kept.', word_wrap=True)
                with ui.HStack(height=32, spacing=10):
                    ui.Button('Cancel', clicked_fn=self.dismiss_defaults)
                    ui.Button('Back up and restore', clicked_fn=self.accept_defaults)

    def dismiss_defaults(self):
        self._confirm_window.visible = False

    def accept_defaults(self):
        self.dismiss_defaults()
        self.request_action('defaults')

    def reset_drone(self, keep_state=None):
        state = keep_state or self.timeline_state()
        self._keys_released = False
        self.backend.set_velocity_command(0, 0, 0, 0)
        self.world.reset()
        for _ in range(60):
            self.world.step(render=False)
        self.backend.ground_height = float(self.drone.state.position[2])
        if state == 'paused':
            self.world.pause()
        elif state == 'stopped':
            self.world.stop()
        self._last_timeline_state = self.timeline_state()

    def process_action(self):
        action, self.pending_action = self.pending_action, None
        if not action:
            return
        if action == 'play':
            if self.world.is_stopped():
                self.reset_drone(keep_state='playing')
            elif not self.world.is_playing():
                self.world.play()
            self.notice = 'Running. Enable the flight keyboard and click the viewport to fly.'
        elif action == 'pause':
            if self.world.is_playing():
                self.backend.set_velocity_command(0, 0, 0, 0)
                self.backend.hold()
                self.world.pause()
            self.notice = 'Paused in place. Play / Resume continues from here.'
        elif action == 'stop':
            self.keyboard_enabled = False
            self.backend.set_velocity_command(0, 0, 0, 0)
            self.world.stop()
            self.notice = 'Stopped. Edit the warehouse, then click Save scene + backup.'
        elif action == 'reset':
            self.reset_drone()
            self.notice = 'Drone reset to the launch pad; timeline state preserved.'
        elif action in ('save', 'defaults'):
            if not self.world.is_stopped():
                self.notice = 'Use Stop / Edit first. Saving during flight is disabled.'
                return
            from scene_storage import save_scene
            saved = save_scene(self.world.stage, self.project_path, stopped=self.world.is_stopped())
            if action == 'defaults':
                from scene_builder import build_scene
                self.metadata.update(build_scene(self.world.stage))
                saved = save_scene(self.world.stage, self.project_path, stopped=self.world.is_stopped())
            self.notice = ('Defaults restored and saved.' if action == 'defaults' else 'Scene saved.')
            if saved['backup']:
                self.notice += '\nPrevious version: backups/' + saved['backup'].replace('\\', '/').rsplit('/', 1)[-1]
        self._keys_released = False

    def update_camera(self):
        import numpy as np
        from isaacsim.core.utils.viewports import set_camera_view
        if self.camera == 'overview':
            eye, target = [-35,-31,20], [0,0,1.5]
        else:
            p = np.asarray(self.drone.state.position)
            yaw = math.radians(self.backend.telemetry()['yaw_deg'])
            direction = np.array([math.cos(yaw), math.sin(yaw), 0])
            eye = p - 4.5*direction + np.array([0,0,2.1])
            target = p + 3*direction + np.array([0,0,0.35])
        set_camera_view(eye=np.asarray(eye), target=np.asarray(target))

    def tick(self):
        import numpy as np
        try:
            self.process_action()
        except Exception as error:
            self.notice = 'Action failed; current project retained: ' + str(error)
            import traceback
            traceback.print_exc()
        state = self.timeline_state()
        if state != self._last_timeline_state:
            if state == 'stopped':
                self.notice = 'Stopped. Edit the warehouse, then click Save scene + backup.'
            elif state == 'paused':
                self.notice = 'Paused in place. Play / Resume continues from here.'
            else:
                self.notice = 'Running. Enable the flight keyboard and click the viewport to fly.'
        if state == 'playing' and self._last_timeline_state == 'stopped':
            # Handle Isaac's native Play button too, but never restart on Stop.
            self.reset_drone(keep_state='playing')
        if state != 'playing':
            self._keys_released = False
        if state == 'stopped':
            self.keyboard_enabled = False
        self._last_timeline_state = state
        command = self.poll_command()
        self.backend.set_velocity_command(*command)
        if state == 'playing' and self.frame % 2 == 0:
            self.update_camera()
        self.frame += 1
        t = self.backend.telemetry()
        p = t['position_m']
        speed = float(np.linalg.norm(t['velocity_m_s']))
        self.status.text = (f"{state.upper()} / {t['mode']}  |  Height {p[2]:.2f} m\n"
                            f"Speed {speed:.2f} m/s  |  Heading {t['yaw_deg']:.0f} deg\n{t['fault']}")
        self.key_status.text = ('KEYBOARD ACTIVE: viewport focused' if self._was_keyboard_active else
                                'KEYBOARD WAITING: click viewport / release keys' if self.keyboard_enabled else
                                'KEYBOARD OFF: editor typing is safe')
        self.keyboard_button.text = 'Disable flight keyboard' if self.keyboard_enabled else 'Enable flight keyboard'
        self.notice_label.text = self.notice
        self.pause_button.enabled = state == 'playing'
        self.play_button.enabled = state != 'playing'
        self.save_button.enabled = self.defaults_button.enabled = state == 'stopped'
        for button in (self.takeoff_button, self.hold_button, self.land_button):
            button.enabled = state == 'playing'
        if time.monotonic() - self.last_log > 1:
            (self.logs/'live_status.json').write_text(json.dumps({
                'scene_ready':True, **t, 'timeline_state':state, 'camera':self.camera,
                'last_key':self.last_key, 'key_press_count':self.key_count,
                'window_focused':self._appwindow.is_focused(),
                'viewport_focused':self.viewport_focused(),
                'keyboard_enabled':self.keyboard_enabled, 'keyboard_active':self._was_keyboard_active,
                'manual_command':command, 'notice':self.notice,
                'scene_path':str(self.project_path)}, ensure_ascii=False, indent=2), encoding='utf-8')
            self.last_log = time.monotonic()



